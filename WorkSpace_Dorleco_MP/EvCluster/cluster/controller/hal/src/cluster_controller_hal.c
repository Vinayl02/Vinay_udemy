/*-----------------------------------------------------------------------------*
 *	OptM Media Solutions Confidential										   *
 *	Copyright (C) OptM Media Solutions Pvt Ltd - All Rights Reserved.	   	   *
 *  Dissemination of this information or reproduction or redistribution of 	   *
 *  this material is  strictly forbidden unless prior written permission is	   *
 *  obtained from OptM Media Solutions Pvt Ltd								   *
 *	File Name	:	cluster_controller_hal.c 							   *
 *  version		: 											              	   *
 *  Date		:	03-Mar-2023												   *
 *  Description :  				 										 	   * 
 *                               						                       *
 *-----------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------*
 * Include Headers				           	                             	   *
 *-----------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "cluster_common.h"
#include "cluster_controller_hal.h"
#include "cluster_controller_hal_config.h"
#include "cluster_controller_hal_bt.h"
#include "cluster_controller_hal_rtc.h"
#include "cluster_controller_hal_switch.h"
#include "cluster_controller_hal_can.h"
#include "cluster_controller_hal_backlight.h"
#include "cluster_controller_hal_flash.h"
#include "cluster_controller_hal_romapi.h"
#include "culster_controller_hal_display.h"
/*----------------------------------------------------------------------------* 
 * Macro ,Structure and enum definition                   				      * 
 *----------------------------------------------------------------------------*/
typedef struct
{
	int32_t				iIsInitialised;
	int32_t				iModulesStatus;
} Hal_t;

/*----------------------------------------------------------------------------* 
 * Static and global variable definition	                         		  * 
 *----------------------------------------------------------------------------*/
static Hal_t hHal = {0};
 
/*----------------------------------------------------------------------------* 
 * Local function definations	                         				      * 
 *----------------------------------------------------------------------------*/
static int32_t Hal_Init_Modules(Hal_t * pHan,int32_t iModules);
static int32_t Hal_Deinit_Modules(Hal_t * pHan,int32_t iModules);


/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Init(int32_t iModules)
{
	int32_t iRval = BCU_OK;
	if(hHal.iIsInitialised ==0)
	{
		memset(&hHal,0, sizeof(Hal_t));
		iRval = Hal_Init_Modules(&hHal, iModules);
		hHal.iIsInitialised = 1;
	}
	return iRval;
}

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_DeInit(int32_t iModules)
{
	int32_t iRval;
	iRval = Hal_Deinit_Modules(&hHal,iModules);
	hHal.iIsInitialised = 0;
	return iRval;
}

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_EnableModule(int32_t iModule,int32_t iEnable)
{
	int32_t iRetVal =0;
	switch(iModule)
	{
		case eModule_BackLight:
		{
			iRetVal = Cluster_Controller_Hal_Backlight_Enable((uint8_t)iEnable);
		}
		break;
		case eModule_Can:
		{
			//iRetVal = Cluster_Controller_Hal_SocketCan_Enable(iEnable);
		}
		break;
		case eModule_Bt:
		{
			//iRetVal = Cluster_Controller_Hal_Bt_Enable(iEnable);
		}
		break;
		case eModule_Rtc:
		{
			//iRetVal = Cluster_Controller_Hal_Rtc_Enable(iEnable);
		}
		break;
		case eModule_Flash:
		{
			//iRetVal =Cluster_Controller_Hal_Flash_Enable(iEnable);
		}
		break;
		default :
		{

		}
		break;
	}
	return iRetVal;
}

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_SetReceiveQueueHandle(int32_t iModule,QueueHandle_t qRxMsg)
{
	int32_t iRetVal = BCU_NOK;
	if(qRxMsg != NULL)
	{
		switch(iModule)
		{
			case eModule_Can:
			{
				iRetVal = Cluster_Controller_Hal_Can_SetReceiveQueueHandle(qRxMsg);
			}
			break;
			case eModule_Switch:
			{
				iRetVal = Cluster_Controller_Hal_Switch_SetReceiveQueueHandle(qRxMsg);
			}
			break;
			case eModule_Bt:
			{
				//iRetVal = Cluster_Controller_Hal_Bt_SetReceiveQueueHandle(qRxMsg);
			}
			break;
			case eModule_Rtc:
			{
				iRetVal = Cluster_Controller_Hal_Rtc_SetReceiveQueueHandle(qRxMsg);
			}
			break;		
			
			default :
			{

			}
			break;			
		
		}
	}
	return iRetVal;
}

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_CanSendMessage(sCanMessage_t *pMsg,uint8_t mbIdx)
{
	return Cluster_Controller_Hal_Can_SendMessage(pMsg,mbIdx);
}

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_CanGetMode(void)
{
	return Cluster_Controller_Hal_Can_GetMode();
}

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_BTSendMessage(int8_t *pMsg,int32_t iMsgLen)
{
	(void)pMsg;
	(void)iMsgLen;
	return 0;//Cluster_Controller_Hal_Bt_SendMessage(pMsg, iMsgLen);
}

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_RtcGetDateTime(sTimeInfo_t * pInfo)
{
	return Cluster_Controller_Hal_Rtc_GetDateTime(pInfo);
}

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_RtcSetDateTime(sTimeInfo_t * pInfo)
{
	return Cluster_Controller_Hal_Rtc_SetDateTime(pInfo);
}

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_RtcUpdateDateTime(void)
{
	return Cluster_Controller_Hal_Rtc_UpdateDateTime();
}
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_SetBrightness(uint8_t iValue)
{
	return Cluster_Controller_Hal_Backlight_SetBrightness(iValue);
}

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_SetBrightnessLevel(uint32_t iLevel)
{
	return Cluster_Controller_Hal_Backlight_SetBrightnessLevel(iLevel);
}

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/

int32_t Cluster_Controller_Hal_FlashWriteData(uint8_t * pBuf, int iSize,uint32_t iAddr)
{
	return Cluster_Controller_Hal_Flash_WriteData(pBuf,iSize,iAddr);
}
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_FlashReadData(uint8_t * pBuf, int iSize,uint32_t iAddr)
{
	return Cluster_Controller_Hal_Flash_ReadData(pBuf,iSize,iAddr);
}
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_FlashErase(uint32_t iAddr)
{
	return Cluster_Controller_Hal_Flash_Erase(iAddr);
}

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_FlashReadProp(int iParam,uint8_t * pBuf, int iSize,uint32_t iAddr)
{
	return  Cluster_Controller_Hal_Flash_ReadProp(iParam,pBuf,iSize,iAddr);
}

/*----------------------------------------------------------------------------*
 * Local function 				                         				      *
 *----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/

static int32_t Hal_Init_Modules(Hal_t * pHan ,int32_t iModules)
{
	int32_t iRval = BCU_NOK;

	if(iModules != 0)
	{
		if(pHan->iModulesStatus  == 0)
		{
			Cluster_Controller_Hal_Config_Init();
		}
		
		if (iModules & eModule_BackLight)
		{
			iRval = Cluster_Controller_Hal_Backlight_Init();
			if(iRval == BCU_OK)
			{
				pHan->iModulesStatus = pHan->iModulesStatus | eModule_BackLight;
			}
			else
			{
				pHan->iModulesStatus = pHan->iModulesStatus & ~eModule_BackLight;
			}
		}
		if (iModules & eModule_Bt)
		{
			/*iRval = Cluster_Controller_Hal_Bt_Init();
			if(iRval == BCU_OK)
			{
				pHan->iModulesStatus = pHan->iModulesStatus| eModule_Bt;
			}
			else
			{
				pHan->iModulesStatus = pHan->iModulesStatus & ~eModule_Bt;
			}*/
		}
		
		if (iModules & eModule_Can)
		{
			iRval = Cluster_Controller_Hal_Can_Init();
			if(iRval == BCU_OK)
			{
				pHan->iModulesStatus = pHan->iModulesStatus| eModule_Can;
			}
			else
			{
				pHan->iModulesStatus = pHan->iModulesStatus & ~eModule_Can;
			}
		}
		
		if (iModules & eModule_Switch)
		{
			iRval = Cluster_Controller_Hal_Switch_Init();
			if(iRval == BCU_OK)
			{
				pHan->iModulesStatus = pHan->iModulesStatus| eModule_Switch;
			}
			else
			{
				pHan->iModulesStatus = hHal.iModulesStatus & ~eModule_Switch;
			}
		}

		if (iModules & eModule_Rtc)
		{
			iRval = Cluster_Controller_Hal_Rtc_Init();
			if(iRval == BCU_OK)
			{
				pHan->iModulesStatus = pHan->iModulesStatus| eModule_Rtc;
			}
			else
			{
				pHan->iModulesStatus = hHal.iModulesStatus & ~eModule_Rtc;
			}
		}

		if (iModules & eModule_Flash)
		{
			iRval = Cluster_Controller_Hal_Flash_Init();
			if(iRval == BCU_OK)
			{
				pHan->iModulesStatus = pHan->iModulesStatus| eModule_Flash;
			}
			else
			{
				pHan->iModulesStatus = hHal.iModulesStatus & ~eModule_Flash;
			}
		}
		if (iModules & eModule_Uds)
		{
			iRval = BCU_OK;
			if(iRval == BCU_OK)
			{
				pHan->iModulesStatus = pHan->iModulesStatus| eModule_Uds;
			}
			else
			{
				pHan->iModulesStatus = hHal.iModulesStatus & ~eModule_Uds;
			}
		}
	}
	return pHan->iModulesStatus;
}

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/

static int32_t Hal_Deinit_Modules(Hal_t * pHan, int32_t iModules)
{
	int32_t iRval = BCU_NOK;

	if(pHan->iModulesStatus != 0)
	{
		if (iModules & eModule_BackLight)
		{
			iRval = Cluster_Controller_Hal_Backlight_DeInit();
			if(iRval == BCU_OK)
			{
				pHan->iModulesStatus = pHan->iModulesStatus & ~eModule_BackLight;
			}
		}
		
		if (iModules & eModule_Bt)
		{
			/*iRval = Cluster_Controller_Hal_Bt_DeInit();
			if(iRval == BCU_OK)
			{
				pHan->iModulesStatus = pHan->iModulesStatus & ~eModule_Bt;
			}*/
		}
		
		if (iModules & eModule_Can)
		{
			//iRval = Cluster_Controller_Hal_Can_DeInit();
			if(iRval == BCU_OK)
			{
				pHan->iModulesStatus =pHan->iModulesStatus & ~eModule_Can;
			}
		}

		if (iModules & eModule_Switch)
		{
			iRval = Cluster_Controller_Hal_Switch_DeInit();
			if(iRval == BCU_OK)
			{
				pHan->iModulesStatus = pHan->iModulesStatus & ~eModule_Switch;
			}
		}

		if (iModules & eModule_Rtc)
		{
			iRval = Cluster_Controller_Hal_Rtc_DeInit();
			if(iRval == BCU_OK)
			{
				pHan->iModulesStatus = pHan->iModulesStatus & ~eModule_Rtc;
			}
		}

		if (iModules & eModule_Flash)
		{
			iRval = Cluster_Controller_Hal_Flash_DeInit();
			if(iRval == BCU_OK)
			{
				pHan->iModulesStatus = pHan->iModulesStatus & ~eModule_Flash;
			}
		}

		if (iModules & eModule_Uds)
		{
			iRval = BCU_OK;
			if(iRval == BCU_OK)
			{
				pHan->iModulesStatus = pHan->iModulesStatus & ~eModule_Uds;
			}
		}

		if(pHan->iModulesStatus  == 0)
		{
			Cluster_Controller_Hal_Config_DeInit();
		}
	}
	return pHan->iModulesStatus;

}

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_CanReset()
{
	return Cluster_Controller_Hal_Can_Reset();
}

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
void Cluster_Controller_Hal_FirmwareDownload_Init()
{
	 Cluster_Controller_Hal_Romapi_FirmwareDownload_Init();
}
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_CanEnable(void)
{
	return Cluster_Controller_Hal_Can_Enable();
}
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_RtcSetMode(bool bSetMode)
{
	return Cluster_Controller_Hal_Rtc_SetMode(bSetMode);
}
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_ConfigGetVersion(char* pBuf,int iSize)
 {
 	return Cluster_Controller_Hal_Config_GetVersion(pBuf ,iSize);
 }
 /*-----------------------------------------------------------------------------*
  * Function	    :											        		   *
  * Params	    :					        		                           *
  * Return value	:									                           *
  * Description	:							                                   *
  *-----------------------------------------------------------------------------*/
 void Cluster_Controller_Hal_ConfigGetSwHwVersion(uint16_t * iSwVersion,uint16_t * iHwVersion)
 {
	 Cluster_Controller_Hal_Config_GetSwHwVersion(iSwVersion, iHwVersion);
 }
 /*-----------------------------------------------------------------------------*
  * Function	    :											        		   *
  * Params	    :					        		                           *
  * Return value	:									                           *
  * Description	:							                                   *
  *-----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_HalDisplay_Enable(bool bOnOff)
 {
 	return Cluster_Controller_Hal_Display_Enable(bOnOff);
 }

