/*-----------------------------------------------------------------------------*
 *	OptM Media Solutions Confidential										   *
 *	Copyright (C) OptM Media Solutions Pvt. Ltd - All Rights Reserved	   	   *
 *																			   *
 *  Dissemination of this information or reproduction or redistribution of 	   *
 *  this material is  strictly forbidden unless prior written permission is	   *
 *  obtained from OptM Media Solutions Pvt Ltd								   *
 *	File Name	:	cluster_controller_hal_config.c							   *
 *  version		: 											              	   *
 *  Date		:	03-Mar-2023												   *
 *  Description :  				 										 	   *
 *                               						                       *
 *																			   *
 *-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------*
 * Include Headers				           	                             	   *
 *-----------------------------------------------------------------------------*/
/* FreeRTOS kernel includes. */
#include "cluster_common.h"
#include "fsl_debug_console.h"
#include <cluster_product_config.h>
#include <fsl_lpuart_freertos.h>
#include "cluster_controller_hal_config.h"
#include "cluster_controller_hal.h"
/*-----------------------------------------------------------------------------*
 * Static and global veriable definations								       *
 *-----------------------------------------------------------------------------*/
typedef struct
{
	int32_t 	iIsInitialised;
}Hal_Config_t;

static Hal_Config_t   hConfig;


/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Config_Init(void)
{
	int32_t iRval = BCU_NOK;

	if(hConfig.iIsInitialised == 0)
	{
		memset(&hConfig, 0 ,sizeof(Hal_Config_t));
	}
	else
	{
		iRval = BCU_OK;
	}
	hConfig.iIsInitialised = 1;
	return iRval;
}

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Config_DeInit(void)
{
	return BCU_OK;
}

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/

int32_t Cluster_Controller_Hal_Config_BtGetInterface(uint32_t* pId, uint32_t* pBaudRate)
{
	int32_t iRetval = BCU_OK;
	if((pId!= NULL) && (pBaudRate != NULL))
	{
		*pId = BT_INTERFACE_ID;
		*pBaudRate = BT_INTERFACE_BAUDRATE;
	}

	return iRetval;
}

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/

int32_t Cluster_Controller_Hal_Config_CanGetInterface(int32_t* pId, uint32_t* pBaudRate)
{
	int32_t iRetval = BCU_OK;
	if((pId!= NULL) && (pBaudRate != NULL))
	{
		*pId = CAN_INTERFACE_ID;
		*pBaudRate = CAN_INTERFACE_BAUDRATE;
	}

	return iRetval;
}

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
uint32_t Cluster_Controller_Hal_Config_GetMaxCanId(void)
{
	return CLUSTER_RX_CANID_MAX;
}
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
sCanConfig_t* Cluster_Controller_Hal_Config_GetCanIdList(void)
{
	return ((sCanConfig_t*)(&Cluster_Rx_CanId_List[0]));
}
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Config_RtcGetInterface(uint32_t * pId, uint32_t * pBaudRate)
{
	int32_t iRetval = BCU_OK;
	if((pId!= NULL) && (pBaudRate != NULL))
	{
		*pId = RTC_INTERFACE_ID;
		*pBaudRate = RTC_INTERFACE_BAUDRATE;
	}

	return iRetval;
}
/*-----------------------------------------------------------------------------*
* Function	    :											        		   *
* Params	    :					        		                           *
* Return value	:									                           *
* Description	:							                                   *
*-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Config_RtcGetInterruptConf(int32_t* pVal)
{
	int32_t iRetval = BCU_OK;

	if(pVal != NULL)
	{
		*pVal = RTC_INTERRUPT_CONFIGURATION;
	}

	return iRetval;
}
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
uint32_t* Cluster_Controller_Hal_Config_SwitchGetList(int Id)
{
	switch(Id)
	{
	case 0 :
		return ((uint32_t*)&GPIO_List[0]);
	case 1 :
		return ((uint32_t*)&CLUSTER_List[0]);
	default:
		break;
	}
	return 0;
}
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Config_BacklightGetInterface(uint8_t* pId, uint32_t* pFreq)
{
	int32_t iRetval = BCU_OK;
	if((pId!= NULL) && (pFreq != NULL))
	{
		*pId	 = BACKLIGHT_PWM_INTERFACE_ID;
		*pFreq 	 = BACKLIGHT_PWM_INTERFACE_FREQUENCE;
	}
	return iRetval;
}
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Config_GetDefaultBrightnessLevel(uint32_t* pVal)
{
	int32_t iRetval = BCU_OK;
	if(pVal != NULL)
	{
		*pVal =(uint32_t)(CLUSTER_BACKLIGHT_LEVEL_DEFAULT);
	}
	else
	{
		iRetval = BCU_NOK;
	}
	return iRetval;
}

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Config_GetMaxBrightnessLevel(uint32_t* pVal)
{
	int32_t iRetval = BCU_OK;
	if(pVal != NULL)
	{
		*pVal =(uint32_t)(CLUSTER_BACKLIGHT_LEVEL_MAX);
	}
	else
	{
		iRetval = BCU_NOK;
	}
	return iRetval;
}
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
uint8_t*  Cluster_Controller_Hal_Config_GetBrightnessMap(void)
{
	return((uint8_t*)(&(Backlight_BrightnessMap[0])));
}
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
void Cluster_Controller_Hal_Config_GetStackSize(uint16_t* pVal)
{
	*pVal =  PRODUCT_SUBMODULE_STACK_SIZE;
}
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
 uint32_t Cluster_Controller_Hal_Config_FlashInterface(uint32_t * pBase,
													  uint32_t * pPCS0,
													  uint32_t * pClk,
													  uint32_t * pSdo,
													  uint32_t * pSdi)
{
	*pBase = (unsigned int)FLASH_GPIO_BASE;
	*pPCS0 = FLASH_PCS0_GPIO_PIN;

	*pClk = FLASH_CLK_GPIO;
	*pSdo = FLASH_SDO_GPIO;
	*pSdi = FLASH_SDI_GPIO;

	return 0;
}

 /*-----------------------------------------------------------------------------*
   Function	    :
   Params	    :
   Return value	:
   Description	:
  *-----------------------------------------------------------------------------*/
  int32_t Cluster_Controller_Hal_Config_GetVersion(char* pBuf,int iSize)
  {
	if(pBuf != NULL)
	{
		snprintf(pBuf,(size_t)(iSize),CLUSTER_VERSION);
	}
  	return 0;
  }
 /*-----------------------------------------------------------------------------*
  * Function	    :											        		   *
  * Params	    :					        		                           *
  * Return value	:									                           *
  * Description	:							                                   *
  *-----------------------------------------------------------------------------*/
uint32_t Cluster_Controller_Hal_Config_GetFlashAddress(uint32_t iSectorAddr)
{
	 return ((iSectorAddr == 0) ? SECTOR_ADDRESS0 :
	            (iSectorAddr == 1) ? SECTOR_ADDRESS1 :
	            (iSectorAddr == 2) ? SECTOR_ADDRESS2 :
	            (iSectorAddr == 3) ? SECTOR_ADDRESS3 :
	                                 SECTOR_ADDRESS4);
}
/*-----------------------------------------------------------------------------*
   Function	    :
   Params	    :
   Return value	:
   Description	:
*-----------------------------------------------------------------------------*/
  int32_t Cluster_Controller_Hal_Config_GetSwHwVersion(uint16_t * iSwVersion, uint16_t * iHwVersion)
  {
	  *iSwVersion = SOFTWARE_VERSION;
	  *iHwVersion = HARDWARE_VERSION;
	  return 0;
  }
#if 0
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Config_GetSystemInfo(ClusterInfo_t* pInfo)
{
	int iRval = BCU_NOK;
	uint8_t sBuf[128] = {0};
	if(pInfo != NULL)
	{
		int iSize = sizeof(ClusterInfo_t);

		iRval = Cluster_Controller_Hal_FlashReadData(&sBuf[0],
													 iSize ,
													 (SYSTEMINFO_SECTOR_NUMBER * SECTOR_SIZE));

		if(iRval == BCU_OK)
		{
			memcpy(pInfo,(ClusterInfo_t*)sBuf,sizeof(ClusterInfo_t));
		}
	}
	return iRval;
}

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Config_UpdateSystemInfo(ClusterInfo_t* pInfo)
{
	int iRval = BCU_NOK;
	uint8_t sBuf[128] = {0};
	if(pInfo != NULL)
	{
		memcpy(sBuf,(uint8_t*)pInfo,sizeof(ClusterInfo_t));
		int iSize = sizeof(ClusterInfo_t);
		iRval = Cluster_Controller_Hal_FlashWriteData(&sBuf[0],
													 iSize ,
													 (SYSTEMINFO_SECTOR_NUMBER * SECTOR_SIZE));
	}
	return iRval;
}
#endif
