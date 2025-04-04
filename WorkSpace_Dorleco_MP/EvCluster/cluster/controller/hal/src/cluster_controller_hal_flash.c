/*---------------------------------------------------------------------------*
 *	OptM Media Solutions Confidential										 *
 *	Copyright (C) OptM Media Solutions Pvt Ltd - All Rights Reserved.	   	 *
 *  Dissemination of this information or reproduction or redistribution of 	 *
 *  this material is  strictly forbidden unless prior written permission is	 *
 *  obtained from OptM Media Solutions Pvt Ltd								 *
 *	File Name	:	cluster_controller_hal_flash.c 							 *
 *  version		: 											              	 *
 *  Date		:	23-May-2023												 *
 *  Description :  				 										 	 * 
 *                               						                     *
 *---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*
 * Include Headers				           	                             	 *
 *---------------------------------------------------------------------------*/
#include "cluster_common.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "cluster_controller_hal_flash.h"
#include "cluster_controller_hal_config.h"
/*----------------------------------------------------------------------------*
 * Macro ,Structure and enum definition                   				      *
 *----------------------------------------------------------------------------*/
#define 	BYTE_MASK  			0xFF

#define 	BYTE_MODE 			3

#define 	BUFFER_SIZE 		(FLASH_SECTOR_SIZE * 8)

#define    tPUW             	10000000       // 10ms
#define    tPP              	5000000        // 5ms
#define    tW               	100000000      // N*15ms, N is a multiple of 10K cycle
#define    tSE              	60000000       // 60ms

#define    CLK_PERIOD      	 	20     			// unit: ns //TBD
#define    MIN_CYCLES     		12 *8  			// 12 Cycles with 8 loops

#define    FLASH_ACCESS_TIME    tPUW / (CLK_PERIOD * MIN_CYCLES)
#define    PAGE_PROGRAM_TIME    tPP  / (CLK_PERIOD * MIN_CYCLES)
#define    SECTOR_ERASE_TIME     tSE / (CLK_PERIOD * MIN_CYCLES)

/*** MX25 series command hex code  ***/
//ID comands
#define    FLASH_CMD_RDID      	0x9F    		//RDID (Read Identification)
#define    FLASH_CMD_RES       	0xAB    		//RES (Read Electronic ID)
#define    FLASH_CMD_REMS      	0x90    		//REMS (Read Electronic & Device ID)

//Register comands
#define    FLASH_CMD_WRSR      	0x01    		//WRSR (Write Status Register)
#define    FLASH_CMD_RDSR      	0x05    		//RDSR (Read Status Register)

//READ comands
#define    FLASH_CMD_READ       0x03    		//READ (1 x I/O)
#define    FLASH_CMD_FASTREAD   0x0B    		//FAST READ (Fast read data)
#define    FLASH_CMD_DREAD      0x3B    		//DREAD (1In/2 Out fast read)
#define    FLASH_CMD_RDSFDP     0x5A    		//RDSFDP (Read SFDP)

//Program comands
#define    FLASH_CMD_WREN     	0x06    		//WREN (Write Enable)
#define    FLASH_CMD_WRDI     	0x04    		//WRDI (Write Disable)
#define    FLASH_CMD_PP       	0x02    		//PP (page program)

//Erase comands
#define    FLASH_CMD_SE       	0x20    		//SE (Sector Erase)
#define    FLASH_CMD_BE       	0xD8    		//BE (Block Erase)
#define    FLASH_CMD_CE       	0x60    		//CE (Chip Erase) hex code: 60 or C7

//Mode setting comands
#define    FLASH_CMD_DP       	0xB9    		//DP (Deep Power Down)
#define    FLASH_CMD_RDP      	0xAB    		//RDP (Release form Deep Power Down)


typedef struct
{
	int32_t 		iIsInitialised;
	uint32_t		iCSPin;
	uint32_t		iClkPin;
	uint32_t		iSdoPin;
	uint32_t		iSdiPin;
	QueueHandle_t   qCmdMsg;
	GPIO_Type *		pGpioBase;
} Flash_t;

/*----------------------------------------------------------------------------* 
 * Static and global variable definition	                         		  * 
 *----------------------------------------------------------------------------*/
static Flash_t hFlash = {0};

//uint8_t sCmdBuf[BUFFER_SIZE];

uint8_t sRawBuff[FLASH_SECTOR_SIZE] = {0};
uint8_t sSendBuff[BUFFER_SIZE];
/*----------------------------------------------------------------------------*
 * Local function definition	                         				      *
 *----------------------------------------------------------------------------*/
static int Flash_IsBusy(void);
static int Flash_WriteEnable(void);
static int Flash_Init(Flash_t* pHan);
static int Flash_IoInit( Flash_t* pHan);
static int Flash_IoConfig( Flash_t* pHan);
static int Flash_CSEnable( Flash_t* pHan, bool bEnable);
static int Flash_GetDeviceID(uint8_t * pBuf, int32_t iSize);
static int Flash_AddDummyCycle( Flash_t* pHan,uint8_t iCount );
static int Flash_ReceiveData(Flash_t* pHan,uint8_t* pBuf, int32_t iSize );
static int Flash_SendData(Flash_t* pHan, uint8_t *  pBuf, int32_t iCmdSize);
static int Flash_GetElectricID(Flash_t* pHan,uint8_t * pBuf, int32_t iSize);
static int Flash_ProcessRead(Flash_t* pHan,
								uint8_t  * pBuf,
								int32_t iSize ,
								 int iCmdSize,
								 uint8_t * pCmd);
/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Flash_Init			     		   *
 * Params	    :	void				        		                       *
 * Return value	:	On success return BCU_OK else it will return BCU_NOK       *
 * Description	:	In this method, Initializes secondary flash(NVM) including
 * 					GPIO's to interface the flash.				               *
 *-----------------------------------------------------------------------------*/
int Cluster_Controller_Hal_Flash_Init()
{
	int iRetVal  = BCU_OK;
	if(hFlash.iIsInitialised == 0)
	{
		hFlash.qCmdMsg = xQueueCreate(1,1);
		Flash_Init(&hFlash);
		hFlash.iIsInitialised = 1;
	}
	return iRetVal;
}
/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Flash_DeInit			     		   *
 * Params	    :	void				        		                       *
 * Return value	:	On success return BCU_OK else it will return BCU_NOK       *
 * Description	:	In this method, Deinitializes secondary flash(NVM).	       *
 *-----------------------------------------------------------------------------*/
 int Cluster_Controller_Hal_Flash_DeInit()
{
	int iRetVal  = BCU_OK;
	if(hFlash.iIsInitialised == 1)
	{
		memset(&hFlash, 0 ,sizeof(Flash_t));
	}
	return iRetVal;
}
 /*-----------------------------------------------------------------------------*
  * Function	    :	Cluster_Controller_Hal_Flash_WriteData			        *
  * Params	    	:	uint8_t* pBuf, int iSize and uint32_t iAddr				*
  * Return value	:	On success return BCU_OK else it will return BCU_NOK    *
  * Description		:	In this method, Write the data into secondary flash
  * 					(NVM) with respect to secondary flash(NVM) manufacturer
  * 					specific.				                                *
  *-----------------------------------------------------------------------------*/
 int  Cluster_Controller_Hal_Flash_WriteData(uint8_t* pBuf,
 										      int iSize,
											  uint32_t iAddr)
 {
 	int iRetVal = BCU_OK;
 	int iCmdSize = 0;

 	if( Flash_IsBusy() == 0)
 	{
 		memset(&sRawBuff[0],0,FLASH_SECTOR_SIZE);

 		Flash_WriteEnable();

 		sRawBuff[iCmdSize] = FLASH_CMD_PP;
 		iCmdSize++;

 		for(int i = 0; i < BYTE_MODE; i++)
 		{
 			sRawBuff[iCmdSize] = (BYTE_MASK & (iAddr));
 			iAddr = iAddr >> 8;
 			iCmdSize =iCmdSize + 1;
 		}

 		for(int i = 0 ; i < iSize; i++)
 		{
 			sRawBuff[iCmdSize+i] = pBuf[i];
 		}

 		iCmdSize = iCmdSize+ iSize;

 		Flash_CSEnable(&hFlash,true);

 		Flash_SendData(&hFlash, &sRawBuff[0], iCmdSize);

 		Flash_CSEnable(&hFlash, false);

 		uint32_t temp = 0;
 		uint32_t ExpectTime = PAGE_PROGRAM_TIME;

 		while( Flash_IsBusy() == -1 )
 		{
 			if( temp > ExpectTime )
 			{
 				PRINTF("\r\n FlashTimeOut \r\n");
 			}
 			temp = temp + 1;
 		}
 	}
 	else
 	{
 		PRINTF("\r\nThe SPI Flash Memory is Busy\r\n");
 	}
 	return iRetVal;
 }

 /*-----------------------------------------------------------------------------*
  * Function	    :	Cluster_Controller_Hal_Flash_Erase			     		*
  * Params	   	    :	int iAddr				        		                *
  * Return value	:	On success return BCU_OK else it will return BCU_NOK    *
  * Description	    :	In this method, Erases the secondary flash(NVM) before
  * 					writing into the flash, and follows the  secondary flash
  * 					(NVM) asper manufacturer specific.				        *
  *-----------------------------------------------------------------------------*/
 int  Cluster_Controller_Hal_Flash_Erase(uint32_t iAddr)
 {
 	int iRetVal = BCU_OK;
 	uint8_t sCmd[8] = {0};
 	int iCmdSize = 0;

    if( Flash_IsBusy() == 0)
 	{
     	Flash_WriteEnable();

     	sCmd[iCmdSize] = FLASH_CMD_SE;
     	iCmdSize++;

     	for(int i = 0; i < BYTE_MODE; i++)
     	{
     		sCmd[iCmdSize] = BYTE_MASK & (iAddr);
     		iAddr = iAddr >> 8;
     		iCmdSize = iCmdSize+1;
     	}

     	Flash_CSEnable(&hFlash,true);

     	Flash_SendData(&hFlash, &sCmd[0], iCmdSize);

     	Flash_CSEnable(&hFlash, false);

     	uint32_t temp = 0;
     	uint32_t ExpectTime = SECTOR_ERASE_TIME;

     	while( Flash_IsBusy() == -1 )
     	{
     		if( temp > ExpectTime )
     		{
     			PRINTF("\r\n FlashTimeOut \r\n");
     		}
     		temp = temp + 1;
     	}
     }
     else
     {
     	PRINTF("\r\nThe SPI Flash Memory is Busy\r\n");
     }
 	return iRetVal;
 }
 /*-----------------------------------------------------------------------------*
  * Function	    :	Cluster_Controller_Hal_Flash_ReadProp			     	*
  * Params	   	    :	int iParam,uint8_t * pBuf,int iSize and int iAddr       *
  * Return value	:	On success return BCU_OK else it will return BCU_NOK    *
  * Description	    :	In this method, Reads the manufacturer data asper
  * 					manufacturer specific.					     		    *
  *-----------------------------------------------------------------------------*/
int  Cluster_Controller_Hal_Flash_ReadProp(int iParam,uint8_t * pBuf,
											int32_t iSize,
											uint32_t iAddr)
{
	(void)iAddr;
	int iRetval = BCU_OK;
	uint8_t sCmd[8] = {0};
	int iCmdSize = 0;

	if((pBuf != NULL) && (iSize >= 0))
	{
		switch(iParam)
		{
			case 0: //MFID
			{
				sCmd[0] = FLASH_CMD_RDID;
				iCmdSize = 1;
				iRetval = Flash_ProcessRead(&hFlash,pBuf,iSize, iCmdSize, &sCmd[0]);
			}
			break;
			case 1: //Electric ID
			{
				iRetval = Flash_GetElectricID(&hFlash,pBuf,iSize);
			}
			break;
			case 2: //Status Register.
			{
				sCmd[0] = FLASH_CMD_RDSR;
				iCmdSize = 1;
				iRetval = Flash_ProcessRead(&hFlash,pBuf,iSize, iCmdSize, &sCmd[0]);
			}
			break;
		   case 3: //Electronic_And_DeviceID
			{
				iRetval = Flash_GetDeviceID(pBuf,iSize);
			}
			break;
		   default:
		   {
			   PRINTF("\r\nCluster_Controller_Hal_Flash_ReadProp : None Case matched\r\n");
		   }
		}
	}
	return iRetval;
}
/*-----------------------------------------------------------------------------*
 * Function	    :	Flash_WriteEnable										   *
 * Params	    :	void				        		                       *
 * Return value	:	On success return BCU_OK else it will return BCU_NOK 	   *
 * Description	:	In this method, write's the write enable command, when
 * 					chip select pin is Low and after writing write enable cmd,
 * 					chip select pin should be high.	  						   *
 *-----------------------------------------------------------------------------*/
static int Flash_WriteEnable(void)
{
	int iCmdSize = 0;
	uint8_t sCmd[4] = {0};

	sCmd[0] = FLASH_CMD_WREN;
	iCmdSize = 1;

	Flash_CSEnable(&hFlash,true);

	Flash_SendData(&hFlash, &sCmd[0], iCmdSize);

	Flash_CSEnable(&hFlash, false);

	return 0;
}
/*-----------------------------------------------------------------------------*
 * Function	    :	Flash_ProcessRead										   *
 * Params	    :	Flash_t* pHan,uint8_t * pBuf,int iSize ,int iCmdSize and
 * 					uint8_t * pCmd				        		               *
 * Return value	:	On success return BCU_OK else it will return BCU_NOK 	   *
 * Description	:	In this method,write's the data into flash when chip select
 * 					is low and after write operation is completed,sets the chip
 * 					select pin as high				                           *
 *-----------------------------------------------------------------------------*/
static int  Flash_ProcessRead(Flash_t* pHan,uint8_t * pBuf,
								 int32_t iSize ,int iCmdSize, uint8_t * pCmd)
{
	int iRetval = BCU_OK;

	if((pBuf != NULL) && (iSize >=0))
	{
		Flash_CSEnable(pHan,true);

		iRetval = Flash_SendData(pHan, pCmd, iCmdSize);
		if(iRetval == BCU_OK)
		{
			Flash_ReceiveData(pHan,&pBuf[0],iSize);
		}
		Flash_CSEnable(&hFlash, false);
	}

	return iRetval;
}
/*-----------------------------------------------------------------------------*
 * Function	    :	Flash_IoConfig									           *
 * Params	    :	Flash_t* pHan				        		               *
 * Return value	:	On success return BCU_OK else it will return BCU_NOK 	   *
 * Description	:	In this method, Configure the GPIO pins asper the specif
 *                  -ication and assigning pins to Flash_t.		               *
 *-----------------------------------------------------------------------------*/
static int Flash_IoConfig( Flash_t* pHan)
{
	uint8_t iRetVal = BCU_OK;

	uint32_t iBase = 0;
	uint32_t iPCS0 = 0;
	uint32_t iClk = 0;
	uint32_t iSdo = 0;
	uint32_t iSdi = 0;

	Cluster_Controller_Hal_Config_FlashInterface(&iBase,
												 &iPCS0,
												 &iClk,
												 &iSdo,
												 &iSdi);
	pHan->pGpioBase = (GPIO_Type *)iBase;
	pHan->iCSPin = iPCS0;

	pHan->iClkPin = iClk;
	pHan->iSdoPin = iSdo;
	pHan->iSdiPin = iSdi;

	return iRetVal;
}
/*-----------------------------------------------------------------------------*
 * Function	    :	Flash_CSEnable									       	   *
 * Params	    :	Flash_t* pHan, bool bEnable				                   *
 * Return value	:	On success return BCU_OK else it will return BCU_NOK 	   *
 * Description	:	In this method, Asper the SPI communication, chip select
 * 					should be low when communicating with slave, once communi
 * 					-cation (Read and write) completed chip select pin should
 * 				    be high. 						                           *
 *-----------------------------------------------------------------------------*/

static int Flash_CSEnable( Flash_t* pHan, bool bEnable)
{
	uint8_t iSet = 0;
	uint8_t iRetVal = BCU_OK;

	if(bEnable == false)
	{
		iSet = 1;
	}
	else
	{
		iSet = 0;
	}

	GPIO_PinWrite(pHan->pGpioBase, pHan->iCSPin, iSet);

	return iRetVal;
}
/*-----------------------------------------------------------------------------*
 * Function	    :	Flash_Init										           *
 * Params	    :	Flash_t* pHan			        		                   *
 * Return value	:	On success return BCU_OK else it will return BCU_NOK	   *
 * Description	:	In this method, initializes the config and gpio's for
 * 					the flash interface						                   *
 *-----------------------------------------------------------------------------*/
static int Flash_Init(Flash_t* pHan)
{
	uint8_t iRetVal = BCU_OK;

	Flash_IoConfig(pHan);

	Flash_IoInit(pHan);

	return iRetVal;
}
/*-----------------------------------------------------------------------------*
 * Function	    :	Flash_IoInit									           *
 * Params	    :	Flash_t* pHan				        		               *
 * Return value	:	On success return BCU_OK else it will return BCU_NOK       *
 * Description	:	In this method, Configure the GPIO pins for input and
 * 					output and initialize the pins with respect to input and
 * 					out put configuration and setting the initial pin values.  *
 *-----------------------------------------------------------------------------*/
static int Flash_IoInit( Flash_t* pHan)
{
	int time_cnt = FLASH_ACCESS_TIME * 2;

	gpio_pin_config_t sGpioOut = {kGPIO_DigitalOutput, 1, kGPIO_NoIntmode};
	gpio_pin_config_t sGpioIn = {kGPIO_DigitalInput, 0, kGPIO_NoIntmode};


	GPIO_PinInit(pHan->pGpioBase, pHan->iCSPin, &sGpioOut);
	GPIO_PinInit(pHan->pGpioBase, pHan->iClkPin, &sGpioOut);
	sGpioOut.outputLogic =1;
	GPIO_PinInit(pHan->pGpioBase, pHan->iSdoPin, &sGpioOut);

	GPIO_PinInit(pHan->pGpioBase, pHan->iSdiPin, &sGpioIn);

	while( time_cnt > 0 )
	{
		time_cnt--;
	}

	return 0;
}
/*-----------------------------------------------------------------------------*
 * Function	    :	Flash_SendData										       *
 * Params	    :	Flash_t* pHan, uint8_t * pBuf, int iSize				   *
 * Return value	:	On success return BCU_OK else it will return BCU_NOK	   *
 * Description	:	In this method, Will write the data into flash through
 * 					SDO in bit by bit along with clock synchronization		   *
 *-----------------------------------------------------------------------------*/
static int  Flash_SendData(Flash_t* pHan, uint8_t * pBuf, int32_t iSize)
{
	 int32_t i;
	 int32_t iRetVal = BCU_OK;
	 uint32_t iCount = 0;

	 memset(sSendBuff,0,BUFFER_SIZE);

	 if((iSize*8) < BUFFER_SIZE)
	 {
		 for(int j = 0; j < iSize; j++)
		 {
			 for( i= 0; i < 8 ; i++ )
			 {
				 if ( (pBuf[j] &  0x80) == 0x80 )
				 {
					 sSendBuff[iCount] =1;
				 }
				 else
				 {
					 sSendBuff[iCount] =0;
				 }

				 pBuf[j] = pBuf[j] << 1;

				 ++iCount;
			 }
		 }

		 for( i= 0; i < iSize ; i++ )
		 {
			 int32_t k = i*8;
			 GPIO_PinWrite(pHan->pGpioBase, pHan->iSdoPin, sSendBuff[k]);

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iClkPin, 0);

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iSdoPin, sSendBuff[k]); // dummy

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iClkPin, 1);

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iSdoPin, sSendBuff[k+1]);

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iClkPin, 0);

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iSdoPin, sSendBuff[k+1]); // dummy

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iClkPin, 1);

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iSdoPin, sSendBuff[k+2]);

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iClkPin, 0);

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iSdoPin, sSendBuff[k+2]); // dummy

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iClkPin, 1);

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iSdoPin, sSendBuff[k+3]);

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iClkPin, 0);

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iSdoPin, sSendBuff[k+3]); // dummy

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iClkPin, 1);

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iSdoPin, sSendBuff[k+4]);

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iClkPin, 0);

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iSdoPin, sSendBuff[k+4]); // dummy

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iClkPin, 1);

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iSdoPin, sSendBuff[k+5]);

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iClkPin, 0);

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iSdoPin, sSendBuff[k+5]); // dummy

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iClkPin, 1);

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iSdoPin, sSendBuff[k+6]);

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iClkPin, 0);

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iSdoPin, sSendBuff[k+6]); // dummy

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iClkPin, 1);

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iSdoPin, sSendBuff[k+7]);

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iClkPin, 0);

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iSdoPin, sSendBuff[k+7]); // dummy

			 GPIO_PinWrite(pHan->pGpioBase, pHan->iClkPin, 1);

		 }
	 }
	 else
	 {
		 iRetVal = BCU_OK;
	 }

	return iRetVal;
}
/*-----------------------------------------------------------------------------*
 * Function	    :	Flash_ReceiveData										   *
 * Params	    :	Flash_t* pHan,uint8_t* pBuf,uint32_t iSize 				   *
 * Return value	:	On success return BCU_OK else it will return BCU_NOK	   *
 * Description	:	In this method, Receives the data in bit by bit through
 * 					SDI	along with clock pin and converting into the byte, by
 * 					keeping the write pin 0.				                   *
 *-----------------------------------------------------------------------------*/
static int  Flash_ReceiveData(Flash_t* pHan,uint8_t* pBuf, int32_t iSize )
{
	uint32_t i;
	uint8_t iByteVal =0;
	uint32_t iBitVal =0;
	uint8_t iRetVal = BCU_OK;
	char data[8]={0};

	for( int j= 0; j < iSize; j++ )
	{
		iByteVal = 0;
		for( i= 0; i < 8; i++ )
		{
			GPIO_PinWrite(pHan->pGpioBase, pHan->iClkPin, 0);

			xQueueReceive(pHan->qCmdMsg,(void *)(&data[0]),(TickType_t)(3));
			iBitVal = GPIO_PinRead(pHan->pGpioBase, pHan->iSdiPin);

			xQueueReceive(pHan->qCmdMsg,(void *)(&data[0]),(TickType_t)(3));
			if(iBitVal == 1)
			{
				iByteVal = (uint8_t)(iByteVal | (0x80 >> i));
			}
			else
			{
				iByteVal = (uint8_t)(iByteVal | (0x00 >> i));  // to keep time delay same
			}
			GPIO_PinWrite(pHan->pGpioBase, pHan->iClkPin, 1);

			xQueueReceive(pHan->qCmdMsg,(void *)(&data[0]),(TickType_t)(3));
		}

		pBuf[j] =iByteVal;
	}
	return iRetVal;
}


/*-----------------------------------------------------------------------------*
 * Function	    :	Flash_GetElectricID										   *
 * Params	    :	Flash_t* pHan,uint8_t * pBuf,int iSize	                   *
 * Return value	:	On success return BCU_OK else it will return BCU_NOK	   *
 * Description	:	In this method, Will send the ElectricID request cmd and
 * 					will receives the ElectricID as a response.                *
 *-----------------------------------------------------------------------------*/
static int Flash_GetElectricID(Flash_t* pHan,uint8_t * pBuf, int32_t iSize)
{
	int iRetval = BCU_OK;

	uint8_t sCmd[4] = {0};

	int iCmdSize = 0;

	if((pBuf != NULL) && (iSize >=0))
	{
		sCmd[0] = FLASH_CMD_RES;
		iCmdSize = 1;

		Flash_CSEnable(pHan,true);

		iRetval = Flash_SendData(pHan, &sCmd[0], iCmdSize);
		if(iRetval == BCU_OK)
		{
			Flash_AddDummyCycle(pHan,24);
			Flash_ReceiveData(pHan,&pBuf[0],iSize);
		}
		Flash_CSEnable(&hFlash, false);
	}

	return iRetval;
}
/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Flash_ReadData	        		   *
 * Params	    :	uint8_t * pBuf,int iSize , uint32_t iAdd                   *
 * Return value	:	On success return BCU_OK else it will return BCU_NOK	   *
 * Description	:	In this method, Receives the data from the flash followed
 * 					by spi protocal and manufacturer specification.			   *
 *-----------------------------------------------------------------------------*/
int  Cluster_Controller_Hal_Flash_ReadData(uint8_t * pBuf,int iSize , uint32_t iAddr)
{
	int iRetval = 0;
    int iCmdSize = 0;

    uint8_t sCmd[4];

    if((pBuf != NULL) && (iSize >=0))
	{
    	sCmd[iCmdSize] = FLASH_CMD_READ;
		iCmdSize =	iCmdSize + 1;
    	for(int i = 0; i < BYTE_MODE; i++)
    	{
    		sCmd[iCmdSize] = iAddr & BYTE_MASK;
    		iAddr = iAddr >> 8;
			iCmdSize = iCmdSize+1;
    	}
		Flash_CSEnable(&hFlash,true);

		iRetval = Flash_SendData(&hFlash, &sCmd[0],iCmdSize);
		if(iRetval == BCU_OK)
		{
			Flash_ReceiveData(&hFlash,&pBuf[0],iSize);
		}

		Flash_CSEnable(&hFlash, false);

	}
	return iRetval;
}
/*-----------------------------------------------------------------------------*
 * Function	    :	Flash_IsBusy										       *
 * Params	    :	void			        		                           *
 * Return value	:	On success return BCU_OK else it will return BCU_NOK	   *
 * Description	:	In this method, If flash is busy while read or write,
 * 				    zeroth index bit will be 1(set), if flash is not busy with
 * 				    read or write, zeroth index will be 0. 					   *
 *-----------------------------------------------------------------------------*/
static int Flash_IsBusy(void)
{
	int32_t iRetval = BCU_OK;
	int32_t iCmdSize = 0;
	uint8_t  sCmd[4] = {0};
	uint8_t  sBuf[4];
    int32_t iSize = 1;

    sCmd[0] = FLASH_CMD_RDSR;
    iCmdSize = 1;
	Flash_ProcessRead(&hFlash,sBuf,iSize,iCmdSize, &sCmd[0]);

	if((sBuf[0]& 0x01) == 0x01)
	{
		iRetval = -1;
	}
	else
	{
		iRetval = 0;
	}
    return iRetval;
}
/*-----------------------------------------------------------------------------*
 * Function	    :	Flash_AddDummyCycle									       *
 * Params	    :	Flash_t* pHan, uint8_t iCount				               *
 * Return value	:	On success return BCU_OK else it will return BCU_NOK	   *
 * Description	:	In this method, will send the dummy clock to warm up the
 * 					flash.						                               *
 *-----------------------------------------------------------------------------*/
static int Flash_AddDummyCycle(Flash_t* pHan, uint8_t iCount )
{
    uint8_t i=0;
    for( i=0; i < iCount; i++ )
    {
        GPIO_PinWrite(pHan->pGpioBase, pHan->iClkPin, 0);

        GPIO_PinWrite(pHan->pGpioBase, pHan->iClkPin, 1);
    }
	return 0;
}
/*-----------------------------------------------------------------------------*
 * Function	    :	Flash_GetDeviceID									       *
 * Params	    :	uint8_t * pBuf, int iSize				                   *
 * Return value	:	On success return BCU_OK else it will return BCU_NOK	   *
 * Description	:	In this method, Send the device id request cmd and will
 * 					receive the device Id as per manufacture specification.	   *
 *-----------------------------------------------------------------------------*/
static int Flash_GetDeviceID(uint8_t * pBuf, int32_t iSize)
{
	int iRetVal = BCU_OK;

	uint8_t sCmd[4] = {0};
	int iCmdSize =0;
    if((pBuf != NULL) && (iSize >=0))
	{
		sCmd[0] = FLASH_CMD_REMS;
		iCmdSize = 1;

		Flash_CSEnable(&hFlash,true);

		Flash_SendData(&hFlash, &sCmd[0],iCmdSize);

		Flash_AddDummyCycle(&hFlash, 16 );

		sCmd[0] = 0;
		iCmdSize = 1;

		Flash_SendData(&hFlash, &sCmd[0], iCmdSize);

		Flash_ReceiveData(&hFlash,&pBuf[0],iSize);

		Flash_CSEnable(&hFlash,false);
	}
	return iRetVal;
}
