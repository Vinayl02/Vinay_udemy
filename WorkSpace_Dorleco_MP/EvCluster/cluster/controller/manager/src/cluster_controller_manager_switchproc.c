/*-----------------------------------------------------------------------------*
 *	OptM Media Solutions Confidential										   *
 *	Copyright (C) OptM Media Solutions Pvt Ltd - All Rights Reserved.	   	   *
 *  Dissemination of this information or reproduction or redistribution of 	   *
 *  this material is  strictly forbidden unless prior written permission is	   *
 *  obtained from OptM Media Solutions Pvt Ltd								   *
 *	File Name	:	cluster_controller_manager.c 							   *
 *  version		: 											              	   *
 *  Date		:	03-Mar-2023												   *
 *  Description :  				 										 	   *
 *                               						                       *
 *-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------*
 * Include Headers				           	                             	   *
 *-----------------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"
#include "fsl_debug_console.h"
#include "clock_config.h"
#include "fsl_gpt.h"
#include "fsl_gpio.h"

#include "cluster_common.h"
#include "cluster_controller_hal.h"
#include "cluster_controller_hal_config.h"
#include "cluster_controller_manager_switchproc.h"
#include "cluster_controller_manager_rtcproc.h"
#include "cluster_controller_manager_canproc.h"
#include "cluster_controller_manager_systeminfo.h"
/*----------------------------------------------------------------------------*
 * Macro ,Structure and Enum definitions           				              *
 *-----------------------------------------------------------------------------*/
#define MAX_QUEUE_MSG_COUNT				10
#define NAV_SWITCH_COUNT				3
#define MSG_BUFFER_SIZE					sizeof(sMessage_t)
#define MSG_QUEUE_BUFFER_SIZE 			MSG_BUFFER_SIZE + 16
#define SWITCH_QUEUE_WAITTIME			portMAX_DELAY
#define SWITCHPROC_GPTIMER              GPT2

typedef struct
{
	bool            bIsTimerEnabled;
	uint32_t       iRefTime;
}SwitchTimer_t;

typedef struct
{
	bool                bIsRtcSetEnabled;
	bool                bIsRtcTimeStarted;
	bool                bIsTripResetEnabled;
	bool                bIsIgnitionOn;
	uint8_t             iCurNavgStatus;
	uint8_t             iPrevNavgStatus;
	uint8_t             iIsDevInfoEnabled;
	int32_t     		iIsInitialised;
	uint32_t            iCurStructCode;
	uint32_t            iTripStartTime;
	uint32_t            iLeftBounceStart;
	uint32_t            iLeftBounceStartTime;
	uint32_t            iRightBounceStart;
	uint32_t            iRightBounceStartTime;

	uint32_t            iOkBounceStart;
	uint32_t            iOkBounceStartTime;
	uint32_t            iInfoStartTime;
	uint32_t            iRtcSetStartTime;
	QueueHandle_t       qRxMsg;
	QueueHandle_t       qRtcMsg;
	QueueHandle_t  	    qHmiMsg;
	TaskHandle_t 	    hRxThread;
	sNavSwitch_t        sNav;
	sHwInputs_t         sHwStatus;
	sAlertInfo_t        sKeyAlertInfo;
	SwitchTimer_t       sTimer[NAV_SWITCH_COUNT];
}SwitchProc_t;

/*----------------------------------------------------------------------------*
 * Static and global variable definition	                         		  *
 *----------------------------------------------------------------------------*/
static SwitchProc_t hSProc = {0};
/*----------------------------------------------------------------------------*
 * Local function definition	                         				      *
 *----------------------------------------------------------------------------*/
static void SwitchProc_CleanUp(void* arg);
static void SwitchProc_MessageHandler(void *arg);
static void SwitchProc_TripReset_TimerConfigure(void);
static void SwitchProc_SendAlertInfo(SwitchProc_t* pHan);
static int32_t SwitchProc_UpdateDeviceInfo(SwitchProc_t* pHan);
static void SwitchProc_UpdateSwitchStruckStatus(SwitchProc_t* pHan);
static void SwitchProc_SendInfoToRtcProc(SwitchProc_t* pHan, uint8_t iKeyVal);
static void SwitchProc_CheckSwitchStuck(SwitchProc_t * pHan,uint8_t iSwStatus);
static uint32_t SwitchProc_GetTimeDiff(	SwitchProc_t* pHan, uint32_t  iPrevTime);
static void SwitchProc_SendProductInfo(SwitchProc_t * pHan, sSystemInfo_t * pInfo);
static void SwitchProc_ExtInput_SendInfo(SwitchProc_t * pHan, sHwInputs_t* pStatus);
static void SwitchProc_Navigation_ParseData(SwitchProc_t * pHan, uint8_t iSwStatus);
static void SwitchProc_Navigation_Process_OkButton(SwitchProc_t * pHan,uint8_t iStatus);
static void SwitchProc_Navigation_Process_LeftButton(SwitchProc_t * pHan,uint8_t iStatus);
static void SwitchProc_Navigation_Process_RightButton(SwitchProc_t * pHan,uint8_t iStatus);
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_SwitchProc_Init(QueueHandle_t QueueId)
{
	int32_t iRval = BCU_NOK;
	if(hSProc.iIsInitialised == 0)
	{
		uint16_t iStackSize =0;
		memset(&hSProc, 0 ,sizeof(hSProc));
		hSProc.qHmiMsg = QueueId;
		hSProc.iCurNavgStatus=0x00;

		Cluster_Controller_Hal_Config_GetStackSize(&iStackSize);

		hSProc.qRxMsg = xQueueCreate(MAX_QUEUE_MSG_COUNT,MSG_QUEUE_BUFFER_SIZE);

		if(hSProc.qRxMsg != 0)
		{
			if (xTaskCreate(SwitchProc_MessageHandler,
							"SwProc",
							iStackSize,
							&hSProc,
							configMAX_PRIORITIES - 2,
							&hSProc.hRxThread) == pdPASS)
			{
				iRval = Cluster_Controller_Hal_SetReceiveQueueHandle(eModule_Switch ,
																	 hSProc.qRxMsg);
				if(iRval == BCU_OK)
				{
					hSProc.iIsInitialised = 1;
					SwitchProc_TripReset_TimerConfigure();
				}
				else
				{
					Cluster_Controller_Manager_SwitchProc_DeInit();
				}
			}
			else
			{
				iRval = BCU_NOK;
				PRINTF("SWITCHPROC:Error: SWITCH_receive_Thread create failed \n");
				SwitchProc_CleanUp(NULL);
			}
		}
		else
		{
			iRval = BCU_NOK;
			PRINTF("SWITCHPROC:Error: xQueueCreate  failed \n");
		}
	}
	return (iRval);
}
/*----------------------------------------------------------------------------*
 * Function	    :													 		  *
 * Params	    :						       							      *
 *			    :   			         		                              *
 * Return value	:														  	  *
 * Description	:								                              *
 *----------------------------------------------------------------------------*/
void Cluster_Controller_Manager_SwitchProc_DeInit(void)
{
	if(hSProc.iIsInitialised)
	{
		vTaskSuspend(hSProc.hRxThread);
		SwitchProc_CleanUp(NULL);
	}
	memset(&hSProc, 0, sizeof(SwitchProc_t));
}
/*----------------------------------------------------------------------------*
 * Function	    :													 		  *
 * Params	    :						       							      *
 *			    :   			         		                              *
 * Return value	:														  	  *
 * Description	:								                              *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_SwitchProc_SetReceiveQueueHandle(QueueHandle_t qRxMsg)
{
	int32_t iRval = BCU_OK;

	if(hSProc.iIsInitialised == 1)
	{
		hSProc.qRtcMsg = qRxMsg;
	}

	return (iRval);
}

/*----------------------------------------------------------------------------*
 * Function	    :													 		  *
 * Params	    :						       							      *
 *			    :   			         		                              *
 * Return value	:														  	  *
 * Description	:								                              *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_SwitchProc_GetHwStatus(sHwInputs_t* pStatus)
{
	int32_t iRval = BCU_OK;

	if(hSProc.iIsInitialised == 1)
	{
		memcpy(pStatus,&(hSProc.sHwStatus),sizeof(sHwInputs_t));
	}

	return (iRval);
}
/*----------------------------------------------------------------------------*
 * Function	    :													 		  *
 * Params	    :						       							      *
 *			    :   			         		                              *
 * Return value	:														  	  *
 * Description	:								                              *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_SwitchProc_RtcSetMode(bool bSetMode)
{
	int32_t iRval = BCU_OK;
	if(bSetMode == false)
	{
		hSProc.bIsRtcSetEnabled = 0;
		hSProc.bIsRtcTimeStarted =0;
	}
	Cluster_Controller_Hal_RtcSetMode(bSetMode);
	return (iRval);
}
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
static void SwitchProc_MessageHandler(void *arg)
{
	SwitchProc_t * pHan = (SwitchProc_t *)(arg);
	char sBuf[MSG_QUEUE_BUFFER_SIZE];
	sMessage_t pSwMsg ;
	sNavSwitch_t  sNavInput ={0};
	sHwInputs_t  sInputs = {0};
	while (1)
	{
		memset(&sBuf[0],0, sizeof(MSG_QUEUE_BUFFER_SIZE));
		memset(&sInputs,0, sizeof(sHwInputs_t));
		memset(&sNavInput,0, sizeof(sNavSwitch_t));

    	if(pdPASS == xQueueReceive(pHan->qRxMsg,(char*)&sBuf[0],SWITCH_QUEUE_WAITTIME))
		{
			memcpy ((char *)&pSwMsg, &(sBuf[0]), sizeof(sMessage_t));

			if(pSwMsg.iEventId == EXTERNAL_SW)
			{
				memcpy((char*)(&sInputs),(char *)pSwMsg.sData,sizeof(sHwInputs_t));
#if CLUSTER_KEYIGNITION_SUPPORTED
				if(pHan->sHwStatus.bInput1 != sInputs.bInput1)
				{
					Cluster_Controller_HalDisplay_Enable(sInputs.bInput1);
					Cluster_Controller_Manager_CanProc_IgnitionStatus(sInputs.bInput1);
					pHan->bIsIgnitionOn = sInputs.bInput1;
				}
#else
				pHan->bIsIgnitionOn  = true;
#endif
				if( (pHan->sHwStatus.bInput2 != sInputs.bInput2) ||
					(pHan->sHwStatus.bInput3 != sInputs.bInput3) ||
					(pHan->sHwStatus.bInput4 != sInputs.bInput4) ||
					(pHan->sHwStatus.bInput5 != sInputs.bInput5) ||
					(pHan->sHwStatus.bInput6 != sInputs.bInput6) ||
					(pHan->sHwStatus.bInput7 != sInputs.bInput7) ||
					(pHan->sHwStatus.bInput8 != sInputs.bInput8) ||
					(pHan->sHwStatus.bInput9 != sInputs.bInput9) ||
					(pHan->sHwStatus.bInput10 != sInputs.bInput10) ||
#if CLUSTER_KEYIGNITION_SUPPORTED					
					(pHan->sHwStatus.bInput1 != sInputs.bInput1) ||	
#endif				
					(sInputs.bRefresh ==1) )
				{
					SwitchProc_ExtInput_SendInfo(pHan,&sInputs);
					memcpy((char*)(&(pHan->sHwStatus)),(char *)pSwMsg.sData,sizeof(sHwInputs_t));

				}
			}
			else if(pSwMsg.iEventId == NAVIGATION_SW)
			{
				if(pHan->bIsIgnitionOn)
				{
					memcpy((char*)(&sNavInput),(char *)pSwMsg.sData,sizeof(sNavSwitch_t));
					SwitchProc_Navigation_ParseData(pHan,sNavInput.iVal);
				}
				else
				{
					if(pHan->bIsRtcSetEnabled == 1)
					{
						SwitchProc_SendInfoToRtcProc(pHan,(uint8_t)(eRtc_Save));
					}
					pHan->bIsRtcSetEnabled = 0;
					pHan->bIsRtcTimeStarted =0;
					pHan->bIsTripResetEnabled = 0;
					pHan->iIsDevInfoEnabled = 0;
				}
			}
			else
			{
				PRINTF("\r\n SWITCH_PROC EVENTID ERROR \r\n");
			}
		}
		taskYIELD();
	}
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
static void SwitchProc_CleanUp(void* arg)
{
	(void)(arg);
	if(hSProc.qRxMsg != 0)
	{
		vQueueDelete(hSProc.qRxMsg);
	}
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
static void SwitchProc_ExtInput_SendInfo(SwitchProc_t * pHan, sHwInputs_t* pStatus)
{
	sMessage_t	  sMsg		= {0};

	if(pHan->qHmiMsg !=NULL)
	{
		sMsg.iEventId 			= eEvent_HwIndicator;
		memcpy((char *)sMsg.sData,(char*)(pStatus),sizeof(sHwInputs_t));
		xQueueSend(pHan->qHmiMsg, &sMsg, (TickType_t)(5000));
	}
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
static void SwitchProc_SendNavigIndicatorInfo(SwitchProc_t* pHan)
{
	sMessage_t	  sMsg		= {0};

	if(pHan->qHmiMsg !=NULL)
	{
		sMsg.iEventId 			= eEvent_NavSwitch;
		memcpy((char *)sMsg.sData,(char*)(&(pHan->sNav)),sizeof(sNavSwitch_t));
		xQueueSend(pHan->qHmiMsg, &sMsg, (TickType_t)(5000));
	}
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
static void SwitchProc_Navigation_ParseData(SwitchProc_t * pHan, uint8_t iSwStatus)
{
	if(pHan != NULL)
	{
		SwitchProc_CheckSwitchStuck(pHan,iSwStatus);
		SwitchProc_Navigation_Process_LeftButton(pHan, iSwStatus);
		SwitchProc_Navigation_Process_OkButton(pHan, iSwStatus);
		SwitchProc_Navigation_Process_RightButton(pHan, iSwStatus);
		pHan->iCurNavgStatus = iSwStatus;
	}
}

/*----------------------------------------------------------------------------*
 * Function	      	:	SwitchProc_DelayMs				          	  	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void SwitchProc_TripReset_TimerConfigure()
{
	gpt_config_t gptConfig = {
				.clockSource = kGPT_ClockSource_Periph,
				.divider = 1,
				.enableRunInStop = false,
				.enableMode =true
		};

	GPT_Init(SWITCHPROC_GPTIMER, &gptConfig);
	GPT_StopTimer(SWITCHPROC_GPTIMER);
	GPT_StartTimer(SWITCHPROC_GPTIMER);
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
static void SwitchProc_SendInfoToRtcProc(SwitchProc_t* pHan, uint8_t iKeyVal)
{
	sMessage_t	  sMsg		= {0};
	pHan->sNav.iVal  = iKeyVal;
	sMsg.iEventId 			= eEvent_NavSwitch;
	memcpy((char *)sMsg.sData,(char*)(&(pHan->sNav)),sizeof(sNavSwitch_t));

	if((pHan->qRtcMsg !=NULL) && (sMsg.iEventId > 0))
	{
		xQueueSend(pHan->qRtcMsg, &sMsg, (TickType_t)(5000));
	}
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
static void SwitchProc_SendProductInfo(SwitchProc_t * pHan, sSystemInfo_t * pInfo)
{
	sMessage_t	  sMsg		= {0};
	if(pHan->qHmiMsg !=NULL)
	{
		sMsg.iEventId 			= eEvent_DeviceInfo;
		memcpy((char *)sMsg.sData,(char*)(pInfo),sizeof(sSystemInfo_t));
		xQueueSend(pHan->qHmiMsg, &sMsg, (TickType_t)(5000));
	}
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
static int32_t SwitchProc_UpdateDeviceInfo(SwitchProc_t* pHan)
{
	int32_t iRval = BCU_NOK;
	sDeviceInfo_t  sInfo = {0};
	sSystemInfo_t  sSysInfo = {0};
	iRval = Cluster_Controller_GetVehicleSpecInfo(&sInfo);
	if(iRval == BCU_OK)
	{
		Cluster_Controller_Hal_Config_GetSwHwVersion(&sSysInfo.iSwVersion, &sSysInfo.iHwVersion);

		sSysInfo.iSlNum = sInfo.iSlNum;

		SwitchProc_SendProductInfo(pHan, &sSysInfo);
	}
	else
	{
		SwitchProc_SendProductInfo(pHan, &sSysInfo);
	}
	return iRval;
}

/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
static void SwitchProc_UpdateSwitchStruckStatus(SwitchProc_t* pHan)
{
	pHan->sKeyAlertInfo.iId = eAlert_Switch;

	if(pHan->sKeyAlertInfo.iCode != pHan->iCurStructCode)
	{
		SwitchProc_SendAlertInfo(pHan);
		pHan->iCurStructCode = pHan->sKeyAlertInfo.iCode;
	}
}
/*----------------------------------------------------------------------------*
 * Function	      	:	SwitchProc_SendAlertInfo 				          	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void SwitchProc_SendAlertInfo(SwitchProc_t* pHan)
{
	sMessage_t	  sMsg		= {0};
	sMsg.iEventId 			= eEvent_Alert;
	memcpy((char *)sMsg.sData,(char*)(&(pHan->sKeyAlertInfo)),sizeof(sAlertInfo_t));
	if((pHan->qHmiMsg !=NULL) && (sMsg.iEventId >0))
	{
		xQueueSend(pHan->qHmiMsg, &sMsg, (TickType_t)(1000));
	}
}

/*----------------------------------------------------------------------------*
 * Function	      	:	SwitchProc_SendAlertInfo 				          	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static uint32_t SwitchProc_GetTimeDiff(	SwitchProc_t* pHan,uint32_t  iRefTime)
{
	uint64_t iMaxTime = 0xFFFFFFFF;
	(void)(pHan);
	uint32_t  iCurTime;
	uint32_t  iTimeDiff;
	iCurTime  = GPT_GetCurrentTimerCount(SWITCHPROC_GPTIMER);

	if(iCurTime > iRefTime)
	{
		iTimeDiff = (iCurTime - iRefTime)/24000;
	}
	else if(iCurTime < iRefTime)
	{
		iTimeDiff =   (uint32_t)(((iMaxTime - iRefTime)+iCurTime)/24000);
	}
	else
	{
		iTimeDiff = 0;
	}
	return iTimeDiff;
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
static void SwitchProc_Navigation_Process_RightButton(SwitchProc_t * pHan,uint8_t iSwStatus)
{
	uint32_t iTimeDiff = 0;

	if((pHan->iCurStructCode & (uint32_t)(eKey_Right)) == 0)
	{
		uint8_t iStatus  = iSwStatus & eKey_Right;
		if((pHan->iCurNavgStatus & eKey_Right) != iStatus)
		{
			if(iStatus == eKey_Right)
			{
				if(pHan->bIsRtcSetEnabled == 1)
				{
					SwitchProc_SendInfoToRtcProc(pHan,(uint8_t)(eRtc_Save));
					pHan->iIsDevInfoEnabled = 0;
					pHan->bIsRtcSetEnabled = 0;
					pHan->bIsRtcTimeStarted =0;
				}
				else
				{
					pHan->iInfoStartTime = GPT_GetCurrentTimerCount(SWITCHPROC_GPTIMER);
					pHan->iIsDevInfoEnabled = 1;
				}
			}
			else
			{
				if(pHan->iIsDevInfoEnabled == 1)
				{
					iTimeDiff = SwitchProc_GetTimeDiff(pHan,pHan->iInfoStartTime);
					if(iTimeDiff >= 4000)
					{
						SwitchProc_UpdateDeviceInfo(pHan);
					}
					else
					{
						if(pHan->iRightBounceStart == 0)
						{
							pHan->iRightBounceStartTime  = GPT_GetCurrentTimerCount(SWITCHPROC_GPTIMER);
							pHan->sNav.iVal = eKey_Right;
							SwitchProc_SendNavigIndicatorInfo(pHan);
							pHan->iRightBounceStart = 1;
						}
						else
						{
							iTimeDiff =  SwitchProc_GetTimeDiff(pHan,pHan->iRightBounceStartTime);
							if(iTimeDiff >= 300)
							{
								pHan->sNav.iVal = eKey_Right;
								SwitchProc_SendNavigIndicatorInfo(pHan);

								pHan->iRightBounceStartTime = GPT_GetCurrentTimerCount(SWITCHPROC_GPTIMER);
							}
						}
					}
				}
				pHan->iIsDevInfoEnabled = 0;
			}
		}
	}
	else
	{
		pHan->iIsDevInfoEnabled = 0;
	}

}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
static void SwitchProc_Navigation_Process_OkButton(SwitchProc_t * pHan,uint8_t iSwStatus)
{
	uint32_t iTimeDiff = 0;

	if((pHan->iCurStructCode & (uint32_t)(eKey_Ok)) == 0)
	{
		uint8_t iStatus  = iSwStatus & eKey_Ok;
		if((pHan->iCurNavgStatus & eKey_Ok)  != iStatus)
		{
			if(iStatus == eKey_Ok)
			{
				if(pHan->bIsRtcTimeStarted == 0)
				{
					pHan->iRtcSetStartTime = GPT_GetCurrentTimerCount(SWITCHPROC_GPTIMER);
					pHan->bIsRtcTimeStarted = 1;
				}
			}
			else
			{
				if(pHan->bIsRtcSetEnabled ==0)
				{
					if(pHan->bIsRtcTimeStarted == 1)
					{
						iTimeDiff = SwitchProc_GetTimeDiff(pHan,pHan->iRtcSetStartTime);
						if(iTimeDiff >= 4000)
						{
							SwitchProc_SendInfoToRtcProc(pHan,(uint8_t)(eRtc_Init));
							pHan->bIsRtcSetEnabled = 1;
						}
						else
						{
							pHan->sNav.iVal = eKey_Ok;
							SwitchProc_SendNavigIndicatorInfo(pHan);
						}
					}

					pHan->bIsRtcTimeStarted = 0;
				}
				else
				{
					if(pHan->iOkBounceStart == 0)
					{
						pHan->iOkBounceStartTime  = GPT_GetCurrentTimerCount(SWITCHPROC_GPTIMER);
						SwitchProc_SendInfoToRtcProc(pHan,(uint8_t)(eRtc_State));
						pHan->iOkBounceStart = 1;
					}
					else
					{
						iTimeDiff =  SwitchProc_GetTimeDiff(pHan,pHan->iOkBounceStartTime);
						if(iTimeDiff >= 300)
						{
							SwitchProc_SendInfoToRtcProc(pHan,(uint8_t)(eRtc_State));
							pHan->iOkBounceStartTime = GPT_GetCurrentTimerCount(SWITCHPROC_GPTIMER);
						}
					}
				}
			}
		}
	}
	else
	{
		if(pHan->bIsRtcSetEnabled == 1)
		{
			SwitchProc_SendInfoToRtcProc(pHan,(uint8_t)(eRtc_Save));
		}
		pHan->bIsRtcSetEnabled = 0;
		pHan->bIsRtcTimeStarted =0;

	}
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
static void SwitchProc_Navigation_Process_LeftButton(SwitchProc_t * pHan,uint8_t iSwStatus)
{
	uint32_t iTimeDiff = 0;


	if((pHan->iCurStructCode & (uint32_t)(eKey_Left)) == 0)
	{
		uint8_t iStatus  = iSwStatus & eKey_Left;

		if((pHan->iCurNavgStatus & eKey_Left)  != iStatus)
		{
			if(pHan->bIsRtcSetEnabled == 0)
			{
				if(iStatus == eKey_Left)
				{
					pHan->bIsTripResetEnabled = 1;
					pHan->iTripStartTime = GPT_GetCurrentTimerCount(SWITCHPROC_GPTIMER);
				}
				else
				{
					if(pHan->bIsTripResetEnabled)
					{
						iTimeDiff =  SwitchProc_GetTimeDiff(pHan,pHan->iTripStartTime);

						if(iTimeDiff >= 4000)
						{
							Cluster_Controller_Manager_CanProc_ResetTrip();
						}
						else
						{
							pHan->sNav.iVal = eKey_Left;
							SwitchProc_SendNavigIndicatorInfo(pHan);
						}
					}
					pHan->bIsTripResetEnabled = 0;
				}
			}
			else
			{
				if(iStatus == eKey_Left)
				{
					if(pHan->iLeftBounceStart == 0)
					{
						pHan->iLeftBounceStartTime  = GPT_GetCurrentTimerCount(SWITCHPROC_GPTIMER);
						SwitchProc_SendInfoToRtcProc(pHan,(uint8_t)(eRtc_Param));
						pHan->iLeftBounceStart = 1;
					}
					else
					{
						iTimeDiff =  SwitchProc_GetTimeDiff(pHan,pHan->iLeftBounceStartTime);
						if(iTimeDiff >= 300)
						{
							SwitchProc_SendInfoToRtcProc(pHan,(uint8_t)(eRtc_Param));
							pHan->iLeftBounceStartTime = GPT_GetCurrentTimerCount(SWITCHPROC_GPTIMER);
						}
					}
				}
			}
		}
	}
	else
	{
		pHan->bIsTripResetEnabled = 0;
	}
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
static void SwitchProc_CheckSwitchStuck(SwitchProc_t * pHan,uint8_t iSwStatus)
{
	uint8_t iKeyval = 0x01;

	int iCurKeyStatus = 0;

	if(pHan->bIsIgnitionOn == 0)
	{
		pHan->sTimer[0].bIsTimerEnabled = 0;
		pHan->sTimer[1].bIsTimerEnabled = 0;
		pHan->sTimer[2].bIsTimerEnabled = 0;
	}
	else
	{
		pHan->sKeyAlertInfo.iCode  = 0;
		for(int i =0; i < NAV_SWITCH_COUNT ; i++)
		{
			iCurKeyStatus = (iSwStatus & (iKeyval << i));

			if(iCurKeyStatus)
			{
				if(pHan->sTimer[i].bIsTimerEnabled == 0)
				{
					pHan->sTimer[i].iRefTime = GPT_GetCurrentTimerCount(SWITCHPROC_GPTIMER);
					pHan->sTimer[i].bIsTimerEnabled = 1;
				}
				else
				{
					uint32_t iTimeDiff = 0;
					iTimeDiff = SwitchProc_GetTimeDiff(pHan,pHan->sTimer[i].iRefTime);
					if(iTimeDiff >= 8000)
					{
						switch(i)
						{
							case 0:
							{
								pHan->sKeyAlertInfo.iCode |= (uint32_t)(eKey_Left);
							}
							break;
							case 1:
							{
								pHan->sKeyAlertInfo.iCode |= (uint32_t)(eKey_Ok);
							}
							break;
							case 2:
							{
								pHan->sKeyAlertInfo.iCode |= (uint32_t)(eKey_Right) ;
							}
							break;
							default:
							break;
						}
					}
				}
			}
			else
			{
				pHan->sTimer[i].bIsTimerEnabled =0;
			}
		}
		SwitchProc_UpdateSwitchStruckStatus(pHan);
	}
}
