/*-----------------------------------------------------------------------------*
 *	OptM Media Solutions Confidential										   *
 *	Copyright (C) OptM Media Solutions Pvt. Ltd - All Rights Reserved	   	   *
 *																			   *
 *  Dissemination of this information or reproduction or redistribution of 	   *
 *  this material is  strictly forbidden unless prior written permission is	   *
 *  obtained from OptM Media Solutions Pvt Ltd								   *
 *	File Name	:	cluster_hal_i2c.c 								  	   		   
 *  version		: 											              	   *
 *  Date		:	03-Mar-2023													   
 *  Description :  				 										 	   * 
 *                               						                       *
 *																			   *
 *-----------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------*
 * Include Headers				           	                             	   *
 *-----------------------------------------------------------------------------*/
#include "fsl_lpi2c_freertos.h"
#include "stdio.h"
#include "fsl_gpio.h"
#include "board.h"
#include "cluster_controller_hal_rtc.h"
#include "cluster_controller_hal_config.h"
#include "fsl_debug_console.h"
/*----------------------------------------------------------------------------* 
 * Macro ,Structure and enum definition                   				      * 
 *----------------------------------------------------------------------------*/
/*Time formate during Time set {0x00, 0x00(sec), 0x55(minute), 0x22(hour),0x00(weeks), 0x04(day), 0x08(month), 0x23(year)}*/

#define 	RTC_ADDR_7BIT 		(0x32U)
#define     CNT_REG   			(16)   //15+1
#define     RESET_BIT_1         (0x01)
#define 	RESET_BIT_0         (0x00)
#define     REG_BUF_SIZE        (17)   //16+1
#define     TIME_INTERVAL       60000 //In milliseconds

#define 	RTC_MAX_QUEUE_MSG_COUNT				1
#define 	RTC_MSG_QUEUE_BUFFER_SIZE           1

typedef struct
{
	uint8_t                     iRtcProcSetMode;
	int32_t						iId;
	int32_t 					iIsRtcSet;
	int32_t 					iIsInitialised;
	uint32_t 					iClkFreq;
	LPI2C_Type*					pBase;
	TaskHandle_t 		  		hIntRxThread;
	QueueHandle_t         		qRxMsg;
	QueueHandle_t         		qHmiRxMsg;
	lpi2c_rtos_handle_t 		hI2c;
	sTimeInfo_t 			    sInfo;
	SemaphoreHandle_t 			pSema;
	sAlertInfo_t				sRtcAlertInfo;
	uint32_t            		iPrevAlertCode;
} sRtc_t;

/*----------------------------------------------------------------------------* 
 * Static and global variable definition	                         		  * 
 *----------------------------------------------------------------------------*/
static sRtc_t	hRtc;
/*----------------------------------------------------------------------------* 
 * Local function definition	                         				      * 
 *----------------------------------------------------------------------------*/
static int32_t RTC_Reset(void);
static void RTC_Update_TimeStamp(void);
static int32_t RTC_Engine(sRtc_t* pHan);
static void RTC_ReceiveThread(void *arg);
static int32_t RTC_Configure(sRtc_t* pHan);
static int32_t RTC_SetData(sRtc_t* pHan ,uint8_t* pBuf, uint8_t iSize) ;
static int32_t RTC_GetData(sRtc_t* pHan ,uint8_t* pBuf, uint8_t iSize) ;
static void RTC_SendAlertInfo(sRtc_t* pHan);
static void Rtc_GetTimeIn12hFormat(sTimeInfo_t * pInfo);
/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Rtc_Init							   *
 * Params	    :	void				        		                       *
 * Return value	:	On success return BCU_OK else return BCU_NOK               *
 * Description	:	In this Method, Initialization and configuration with respe
 * 					-ct to RTC module will take place including the RTC reset,
 * 				    And if the RTC module has battery support, it will skip the
 * 				    rtc reset and calls the RTC_Engine.					       *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Rtc_Init(void)
{
	int32_t iRval = BCU_NOK;

	if(hRtc.iIsInitialised == 0)
	{
		memset(&hRtc, 0 ,sizeof(sRtc_t));
		hRtc.pSema = xSemaphoreCreateBinary();

		if((RTC_Configure(&hRtc) == BCU_OK)
			&&(RTC_Engine(&hRtc) == BCU_OK)
			&& (hRtc.pSema != NULL))
		{
			xSemaphoreGive(hRtc.pSema);
			hRtc.iIsInitialised = 1;
			iRval = BCU_OK;

		}
		else
		{
			PRINTF("\r\nERROR: RTC Initialization Failed\r\n");
		}
	}
	return iRval;
}

/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Rtc_DeInit						   *
 * Params	    :	void				        		                       *
 * Return value	:	On success return BCU_OK else return BCU_NOK			   *
 * Description	:	In this Method, Denitialization of RTC will take place alo
 * 					-ng with deletion queue and thread whith is created during
 * 					the initialization.					                       *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Rtc_DeInit(void)
{
	int32_t iRval = BCU_NOK;
	if(hRtc.iIsInitialised == 1)
	{
		if(  hRtc.hIntRxThread != NULL )
		{
			vTaskDelete(hRtc.hIntRxThread);
			vQueueDelete(hRtc.qRxMsg);
		}

		iRval = BCU_OK;
	}
	return iRval;
}

/*----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Rtc_Enable						  *
 * Params	    :	int32_t iEnable			        		                  *
 * Return value	:	On success return BCU_OK else return BCU_NOK		      *
 * Description	:	In this method, Enable and disable of RTC module will be
 * 				    take place.								                  *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Rtc_Enable(int32_t iEnable)
{
	(void)iEnable;
	int32_t iRval = BCU_NOK;
	if(hRtc.iIsInitialised == 1)
	{
		iRval = BCU_OK;
	}
	return iRval;
}
/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Rtc_SetReceiveQueueHandle		   *
 * Params	    :	QueueHandle_t  hHmiQueue				        		   *
 * Return value	:	On success return BCU_OK else return BCU_NOK			   *
 * Description	:	In this method, accessing the Hmi queue Id is take place
					to send the parsed data.						           *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Rtc_SetReceiveQueueHandle(QueueHandle_t  hHmiQueue)
{
	int32_t iRval = BCU_OK;
	if((hHmiQueue != 0) && (hRtc.iIsInitialised == 1))
	{
		hRtc.qHmiRxMsg = hHmiQueue;
	}
	return iRval;
}

/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Rtc_GetDateTime					   *
 * Params	    :	uint8_t* pBuf, uint8_t iSize				               *
 * Return value	:	On success return BCU_OK else return BCU_NOK			   *
 * Description	:   In this method, gets the data from RTC module via i2c.	   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Rtc_GetDateTime(sTimeInfo_t * pDate)
{
	int32_t iRval = BCU_NOK;
	if((hRtc.iIsInitialised == 1) && (pDate != NULL))
	{

		xSemaphoreTake(hRtc.pSema,0xFFFF);

		pDate->bPM		= hRtc.sInfo.bPM;
		pDate->iDay		= hRtc.sInfo.iDay;
		pDate->iMonth	= hRtc.sInfo.iMonth;
		pDate->iYear	= hRtc.sInfo.iYear;
		pDate->iMinute	= hRtc.sInfo.iMinute;
		pDate->iHour	= hRtc.sInfo.iHour;

		xSemaphoreGive(hRtc.pSema);
	}
	return iRval;
}

/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Rtc_UpdateDateTime					   *
 * Params	    :	uint8_t* pBuf, uint8_t iSize				               *
 * Return value	:	On success return BCU_OK else return BCU_NOK			   *
 * Description	:  	   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Rtc_UpdateDateTime(void)
{
	int32_t iRval = BCU_OK;

	if(hRtc.iIsInitialised == 1)
	{
		RTC_Update_TimeStamp();
	}
	return iRval;
}
/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Rtc_SetDateTime					   *
 * Params	    :	uint8_t* pBuf, uint8_t iSize				        	   *
 * Return value	:	On success return BCU_OK else return BCU_NOK		       *
 * Description	:	In this method, The Api is exposed to CAN to set the data
 * 					and get the data from the rtc, and also if from the any mo
 * 					-dule wants change the time, can use this method.		   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Rtc_SetDateTime(sTimeInfo_t * pRtcInfo)
{
	int32_t iRval = BCU_NOK;
	uint8_t sBuf[8] = {0};

	if((hRtc.iIsInitialised == 1) && (pRtcInfo != NULL))
	{
		hRtc.iIsRtcSet = 1;

		sBuf[0] = 0x00;
		sBuf[1] = (uint8_t)(((pRtcInfo->iSecond/10) <<4) |(pRtcInfo->iSecond%10));
		sBuf[2] = (uint8_t)(((pRtcInfo->iMinute/10) <<4) |(pRtcInfo->iMinute%10));
		sBuf[3] = (uint8_t)(((pRtcInfo->iHour/10) <<4) |(pRtcInfo->iHour%10));
		sBuf[4] = 0x00;
		sBuf[5] = (uint8_t)(((pRtcInfo->iDay/10) <<4) |(pRtcInfo->iDay%10));
		sBuf[6] = (uint8_t)(((pRtcInfo->iMonth/10) <<4) |(pRtcInfo->iMonth%10));
		sBuf[7] = (uint8_t)(((pRtcInfo->iYear/10) <<4) |(pRtcInfo->iYear%10));


		iRval = RTC_Reset();
		if(iRval == BCU_OK)
		{
			iRval = RTC_SetData(&hRtc,sBuf,8);
			if(iRval != BCU_OK)
			{
				PRINTF("\r\n ERROR: Time set failed\r\n");
			}
		}
		if(iRval == BCU_OK)
		{
			memcpy(&hRtc.sInfo,pRtcInfo,sizeof(sTimeInfo_t));
			hRtc.sInfo.iIsEdit = 0;
			hRtc.sInfo.iState  = 0;
			Rtc_GetTimeIn12hFormat(&hRtc.sInfo);
		}
		hRtc.iIsRtcSet = 0;
	}
	return iRval;
}

/*-----------------------------------------------------------------------------*
 * Function	    : Cluster_Controller_Hal_Rtc_SetMode	                       *
 * Params	    : bool bMode			        		                       *
 * Return value	:                                                   		   *
 * Description	:                                                              *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Rtc_SetMode(bool bMode)
{
	int32_t iRval = BCU_OK;

	if (hRtc.iIsInitialised == 1)
	{
		hRtc.iRtcProcSetMode = (bMode == true)? 1: 0;
	}

	return iRval;
}

/*-----------------------------------------------------------------------------*
 * Function	    : RTC_Configure											       *
 * Params	    : sRtc_t* pHan			        		                       *
 * Return value	: On success return BCU_OK else return BCU_NOK				   *
 * Description	: in this method, configuration of i2c is take place along with
 * 				  LPI2C_RTOS_Init api initialization for RTC communication.    *
 *-----------------------------------------------------------------------------*/ 
static int32_t RTC_Configure(sRtc_t* pHan)
{
	int32_t iRetVal = BCU_OK;
	uint32_t iId;
	uint32_t iBaudRate;
	lpi2c_master_config_t sRtcConfig = {0};

	iRetVal = Cluster_Controller_Hal_Config_RtcGetInterface(&iId, &iBaudRate);
	if(iRetVal == BCU_OK)
	{
		status_t status;
		pHan->pBase = NULL;

		switch(iId)
		{
			case 1:
			{
				pHan->pBase = LPI2C1;
				NVIC_SetPriority(LPI2C1_IRQn, 3);
			}
			break;
			case 2:
			{
				pHan->pBase = LPI2C2;
				NVIC_SetPriority(LPI2C2_IRQn, 3);
			}
			break;
			case 3:
			{
				pHan->pBase = LPI2C3;
				NVIC_SetPriority(LPI2C3_IRQn, 3);
				break;
			}
			case 4:
			{
				pHan->pBase = LPI2C4;
				NVIC_SetPriority(LPI2C4_IRQn, 3);
				break;
			}
			default:
			{
				pHan->pBase = LPI2C4;
				NVIC_SetPriority(LPI2C4_IRQn, 3);
			}
			break;
		}

		LPI2C_MasterGetDefaultConfig(&sRtcConfig);
		sRtcConfig.baudRate_Hz = iBaudRate;

		pHan->iClkFreq = CLOCK_GetFreq(kCLOCK_OscRc48MDiv2);

		status = LPI2C_RTOS_Init(&(pHan->hI2c), pHan->pBase, &sRtcConfig, pHan->iClkFreq);
		if (status != kStatus_Success)
		{
			iRetVal = BCU_NOK;
		}
	}

	return iRetVal;
}

/*-----------------------------------------------------------------------------*
 * Function	    :	RTC_GetData										           *
 * Params	    :	sRtc_t* pHan ,uint8_t* pBuf, uint8_t iSize				   *
 * Return value	:	On success return BCU_OK else return BCU_NOK			   *
 * Description	:	In this method, Getting of data from rtc will take place
 * 					through i2c.						                       *
 *-----------------------------------------------------------------------------*/
static int32_t RTC_GetData(sRtc_t* pHan ,uint8_t* pBuf, uint8_t iSize)
{
	int32_t iRval = BCU_NOK;
	lpi2c_master_transfer_t rtcXfer;

	if((pBuf != NULL) && (iSize > 0))
	{
		memset(pBuf, 0, sizeof(iSize));

		memset(&rtcXfer, 0, sizeof(rtcXfer));
		rtcXfer.slaveAddress   = RTC_ADDR_7BIT;
		rtcXfer.direction      = kLPI2C_Read;
		rtcXfer.subaddress     = 0;
		rtcXfer.subaddressSize = 0;
		rtcXfer.data           = pBuf;
		rtcXfer.dataSize       = iSize;
		rtcXfer.flags          = kLPI2C_TransferDefaultFlag;

		iRval = LPI2C_RTOS_Transfer(&(pHan->hI2c), &rtcXfer);
		if(iRval == kStatus_Success)
		{
			iRval = BCU_OK;
		}
	}

	return iRval;
}
/*-----------------------------------------------------------------------------*
 * Function	    :	RTC_SetData										           *
 * Params	    :	sRtc_t* pHan ,uint8_t* pBuf, uint8_t iSize				   *
 * Return value	:	On success return BCU_OK else return BCU_NOK               *
 * Description	:	In this method, Getting of data from rtc will take place
 * 					through i2c.						                       *
 *-----------------------------------------------------------------------------*/
static int32_t RTC_SetData(sRtc_t* pHan ,uint8_t* pBuf, uint8_t iSize)
{
	int32_t iRval = BCU_NOK;
	lpi2c_master_transfer_t rtcXfer;

	if((pBuf != NULL) && (iSize > 0))
	{
		memset(&rtcXfer, 0, sizeof(rtcXfer));
		rtcXfer.slaveAddress   = RTC_ADDR_7BIT;
		rtcXfer.direction      = kLPI2C_Write;
		rtcXfer.subaddress     = 0;
		rtcXfer.subaddressSize = 0;
		rtcXfer.data           = pBuf;
		rtcXfer.dataSize       = iSize;
		rtcXfer.flags          = kLPI2C_TransferDefaultFlag;

		iRval = LPI2C_RTOS_Transfer(&(pHan->hI2c), &rtcXfer);

		if(iRval == kStatus_Success)
		{
			iRval = BCU_OK;
		}
	}

	return iRval;
}
/*-----------------------------------------------------------------------------*
 * Function	    :	RTC_Reset										           *
 * Params	    :	void				        		                       *
 * Return value	:	On success return BCU_OK else return BCU_NOK               *
 * Description	:	In this method, Reset of rtc module will take place with re
 *   				-spect to rtc module data sheet.						   *
 *-----------------------------------------------------------------------------*/
static int32_t RTC_Reset(void)
{
	int32_t iRval = BCU_NOK;

	unsigned char sBuf[REG_BUF_SIZE] = {0};

	sBuf[CNT_REG] = RESET_BIT_1;

	iRval = RTC_SetData(&hRtc,&sBuf[0],REG_BUF_SIZE);
	if(iRval == BCU_OK)
	{
		memset(sBuf, 0, sizeof(sBuf));
		sBuf[CNT_REG] = RESET_BIT_0;
		iRval = RTC_SetData(&hRtc,&sBuf[0],REG_BUF_SIZE);
		if(iRval != BCU_OK)
		{
			PRINTF("\r\nERROR: RESET_BIT_0\r\n");
		}
	}
	else
	{
		PRINTF("\r\nERROR:RESET_BIT_1\r\n");
	}

	return iRval;
}
/*-----------------------------------------------------------------------------*
 * Function	    :  RTC_ReceiveThread					            		   *
 * Params	    :  void *arg				        		                   *
 * Return value	:  void								                           *
 * Description	:  In this method, Gets the data from rtc module, based on every
 * 				   minute. 				                                       *
 *-----------------------------------------------------------------------------*/
static void RTC_ReceiveThread(void *arg)
{
	sRtc_t *  pHan = (sRtc_t *)(arg);
	unsigned char sQueueBuf[REG_BUF_SIZE] = {0};
	TickType_t tWait = 3000;
	while(1)
	{
		xQueueReceive(pHan->qRxMsg,&sQueueBuf[0],tWait); // for wait only
		if((hRtc.iIsRtcSet == 0) && (hRtc.iRtcProcSetMode == 0))
		{
			RTC_Update_TimeStamp();
			tWait = TIME_INTERVAL;
		}
		else
		{
			tWait = 2000;
		}
	}
}
/*-----------------------------------------------------------------------------*
 * Function	    :	RTC_Update_TimeStamp							  		   *
 * Params	    :	void			        		                           *
 * Return value	:	void								                       *
 * Description	:	In this method, after getting the data from rtc, data parse
 * 					is required, so the data parsing for time and date will take
 * 					place with respect data sheet and updating it into HMI queue
 * 					.   													   *
 *-----------------------------------------------------------------------------*/
static void RTC_Update_TimeStamp(void)
{
	xSemaphoreTake(hRtc.pSema,0xFFFF);

	int32_t iRval = BCU_NOK;
	sMessage_t sMsg		= {0};
	unsigned char sBuf[REG_BUF_SIZE] = {0};

	iRval = RTC_GetData(&hRtc,&sBuf[0],(REG_BUF_SIZE-1));
	if(iRval == BCU_OK)
	{
		if((sBuf[1] == 0xFF) && (sBuf[2] == 0xFF))
		{
           memset(&(hRtc.sInfo),0,sizeof(hRtc.sInfo));
           hRtc.sInfo.bEnable = false;
           hRtc.sRtcAlertInfo.iCode = eRTC_NotSetOrBtryWeak;
		}
		else if((sBuf[1] == 0) && (sBuf[2] == 0)
				&& (sBuf[4] == 0) && (sBuf[5] == 0))
		{
			memset(&(hRtc.sInfo),0,sizeof(hRtc.sInfo));
			hRtc.sInfo.bEnable = false;
			hRtc.sRtcAlertInfo.iCode = eRTC_NotWorking;
		}
		else
		{
			hRtc.sRtcAlertInfo.iCode = eRTC_Working;
			hRtc.sInfo.bEnable = true;
			hRtc.sInfo.iMinute = (uint8_t)((sBuf[1] & 0x0F) + (((sBuf[1] & 0xF0) >> 4) *10));
			hRtc.sInfo.iHour   = (uint8_t)((sBuf[2] & 0x0F) + (((sBuf[2] & 0xF0) >> 4) *10));
			hRtc.sInfo.iDay    = (uint8_t)((sBuf[4] & 0x0F) + (((sBuf[4] & 0xF0) >> 4) *10));
			hRtc.sInfo.iMonth  = (uint8_t)((sBuf[5] & 0x0F) + (((sBuf[5] & 0xF0) >> 4) *10));
			hRtc.sInfo.iYear   = (uint8_t)((sBuf[6] & 0x0F) + (((sBuf[6] & 0xF0) >> 4) *10));
			hRtc.sInfo.bPM	   = false;

			Rtc_GetTimeIn12hFormat(&hRtc.sInfo);
		}

		sMsg.iEventId 			= eEvent_Time;
		memcpy((char *)sMsg.sData,(char*)(&(hRtc.sInfo)),sizeof(sTimeInfo_t));

		if((hRtc.qHmiRxMsg  != NULL) && (sMsg.iEventId > 0))
		{
			xQueueSend(hRtc.qHmiRxMsg, &sMsg, 100);
		}

		if(hRtc.iPrevAlertCode != hRtc.sRtcAlertInfo.iCode)
		{
			RTC_SendAlertInfo(&hRtc);
			hRtc.iPrevAlertCode = hRtc.sRtcAlertInfo.iCode;
		}
	}
	xSemaphoreGive(hRtc.pSema);
}
/*-----------------------------------------------------------------------------*
 * Function	    :	RTC_Engine										           *
 * Params	    :	sRtc_t* pHan				        		               *
 * Return value	:	On success return BCU_OK else return BCU_NOK               *
 * Description	:	                                                           *
 *-----------------------------------------------------------------------------*/
static void Rtc_GetTimeIn12hFormat(sTimeInfo_t * pInfo)
{
	if(pInfo != NULL)
	{
		if(pInfo->iHour >= 12)
		{
			pInfo->bPM = true;
			if(pInfo->iHour > 12)
			{
				pInfo->iHour = pInfo->iHour - 12;
			}
		}
		else if(pInfo->iHour == 0)
		{
			pInfo->bPM = false;
			pInfo->iHour = 12;
		}
	}
}
/*-----------------------------------------------------------------------------*
 * Function	    :	RTC_Engine										           *
 * Params	    :	sRtc_t* pHan				        		               *
 * Return value	:	On success return BCU_OK else return BCU_NOK               *
 * Description	:	In this method, Creation of thread and queue is take place
 * 					to get the data in polling mode with time delay of 60000ms.*
 *-----------------------------------------------------------------------------*/
static int32_t RTC_Engine(sRtc_t* pHan)
{
	int32_t iRval = BCU_OK;
	uint16_t iStackSize = 0;

	pHan->qRxMsg = xQueueCreate(RTC_MAX_QUEUE_MSG_COUNT,
								RTC_MSG_QUEUE_BUFFER_SIZE);

	if(pHan->qRxMsg  != 0)
	{
		Cluster_Controller_Hal_Config_GetStackSize(&iStackSize);
		if (pdPASS != xTaskCreate(	RTC_ReceiveThread,
									"RTC",
									iStackSize,
									pHan,
									(configMAX_PRIORITIES - 2),
									&pHan->hIntRxThread))
		{
			iRval = BCU_NOK;
		}
	}
	else
	{
		iRval = BCU_NOK;
		PRINTF("\r\n ERROR: RTC xQueueCreate\r\n");
	}

	return iRval;
}
/*----------------------------------------------------------------------------*
 * Function	      	:	RTC_SendAlertInfo		 				          	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void RTC_SendAlertInfo(sRtc_t* pHan)
{
	sMessage_t	  sMsg		= {0};
	sMsg.iEventId 			= eEvent_Alert;

	pHan->sRtcAlertInfo.iId = eAlert_Rtc;
	memcpy((char *)sMsg.sData,(char*)(&(pHan->sRtcAlertInfo)),sizeof(sAlertInfo_t));

	if((pHan->qHmiRxMsg !=NULL) && (sMsg.iEventId >0))
	{
		xQueueSend(pHan->qHmiRxMsg, &sMsg, (TickType_t)(5000));
	}
}
