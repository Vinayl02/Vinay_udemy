/*-----------------------------------------------------------------------------*
 *	OptM Media Solutions Confidential										   *
 *	Copyright (C) OptM Media Solutions Pvt Ltd - All Rights Reserved.	   	   *
 *  Dissemination of this information or reproduction or redistribution of 	   *
 *  this material is  strictly forbidden unless prior written permission is	   *
 *  obtained from OptM Media Solutions Pvt Ltd								   *
 *	File Name	:	cluster_controller_manager_canproc.c					   *
 *  version		: 											              	   *
 *  Date		:	03-Mar-2023												   *
 *  Description :  				 										 	   *
 *                               						                       *
 *-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*
 * Include Headers				           	                             	   *
 *-----------------------------------------------------------------------------*/
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"
#include "fsl_debug_console.h"
#include "clock_config.h"
#include "fsl_flexcan.h"
#include "fsl_gpt.h"

#include "cluster_common.h"
#include "cluster_controller_hal.h"
#include "cluster_controller_hal_config.h"
#include "cluster_controller_manager_canproc.h"
#include "cluster_controller_manager_systeminfo.h"
#include "cluster_controller_hal_rtc.h"
#include "cluster_controller_hal_backlight.h"
#include "cluster_controller_hal_can.h"
#include "cluster_controller_manager_rtcproc.h"
#include "cluster_controller_manager_switchproc.h"
#include "cluster_controller_manager_udsproc.h"
/*----------------------------------------------------------------------------*
 * Macro ,Structure and Enum definitions                        				  *
 *-----------------------------------------------------------------------------*/
#define CAN_MAX_QUEUE_MSG_COUNT				150
#define CAN_MSG_BUFFER_SIZE					sizeof(sCanMsg_t)
#define READY_STATE_WAIT_TIME     			3000
#define MSG_RECEIVE_INIT_WAIT				1000
#define MSG_RECEIVE_ERROR_WAIT              10000
#define MSG_RECEIVE_DEFAULT_WAIT			25000
#define MSG_SEND_DEFAULT_WAIT				5000
#define CAN_SEND_MBID						25
#define SPEED_CACHE_SIZE					2
#define FLASH_MODE                          0xF1
#define RTC_SET_TIME                        0xF2
#define FACTORY_RESET                    	0xF3
#define BACKLIGHT_SET                       0xF4
#define CANPROC_GPTIMER                 	GPT1
#define HARDWARE_STATUS                     0xF5
#define READ_SWHW_VERSION                   0xF6
#define PRODUCT_SWHW_VERSION                0xF7
#define VINFO_WRITE_START           		0xF8
#define VINFO_WRITE_CONTINUE				0xF9
#define VINFO_WRITE_END     			    0xFA
#define VINFO_READ     			            0xFB
#define VINFO_READ_CONTINUE  				0xFC
#define VINFO_READ_END       				0xFD
#define SET_ODO                             0xFE

#define SLN_UMBER                           0x01
#define VIN                                 0x02
#define DATE                                0x04



typedef struct
{
	bool				bCanError;
	bool	        	bPrevCoolant;
	bool	        	bIsOdoCached;
	bool                bIsResetDist;
	bool                bIsSpeedUpdate;
	int32_t     		iIsInitialised;
	int8_t				iSendCmdflag;
	uint8_t             sInfoCacheData[32];
	int32_t             iSOCUpdateCount;
	int32_t             iFaultFlag;
	uint64_t			iCurTrip;
	uint64_t 			iCurOdo;
	uint64_t 			iInitialOdo;
	uint64_t 			iPrevOdo;
	uint64_t 			iCurOdoDiff;
	uint32_t  			iCurTime;
	uint32_t   			iPrevTime;
	uint32_t	        iCurIndex;
	double				iSpeed[SPEED_CACHE_SIZE];
	uint64_t			iTimeOffset[SPEED_CACHE_SIZE];
	int32_t 			iGearMode;
	int32_t 			iDriveMode;
	uint32_t 			iCurRpm;
	uint32_t 			iCurPower;
	uint32_t 			iCurRange;
	uint32_t			iCurSpeed;
	uint32_t	        iCurBatTemp;
	uint32_t	        iChargerStatus;
	uint32_t	        iChargeTime;
	uint32_t			iPrevBatteryFault;
	uint32_t			iPrevPTStatus;
	QueueHandle_t   	qRxMsg;
	QueueHandle_t   	qHmiMsg;
	QueueHandle_t   	qCmdMsg;
	QueueHandle_t   	qUdsRxMsg;
	TaskHandle_t 		hRxThread;
	TaskHandle_t 		hCmdThread;
	sMotorInfo_t 		sMotor;
	sBatteryInfo_t 		sBattery;
	sDriveMode_t  		sDriveMode;
	sChargeInfo_t     	sCharge;
	sFault_t			sFault[5];
	sStatus_t           sCanStatus;
	sHwInputs_t         sHwStatus;
	sDeviceInfo_t       sSysInfo;
	sDeviceInfo_t       sSysInfoCache;
}CanProc_t;
/*----------------------------------------------------------------------------*
 * Static and global variaCan definition	                         		  *
 *----------------------------------------------------------------------------*/
static CanProc_t hCanProc = {0};
/*----------------------------------------------------------------------------*
 * Local function definition	                         				      *
 *----------------------------------------------------------------------------*/
static void CanProc_CleanUp(void* arg);
static void CanProc_TimerConfigure(void);
static void CanProc_DataHandler(void *arg);
static int32_t CanProc_ProductionReset(void);
static void CanProc_TripReset(CanProc_t* pHan);
static uint32_t CanProc_GetCurrentTimestamp(void);
static void CanProc_SendCommandHandler(void *arg);
static void CanProc_SendMotorInfo(CanProc_t* pHan);
static int  CanProc_UpdateDistance(CanProc_t* pHan);
static void CanProc_Send_DummyData(CanProc_t* pHan);
static void CanProc_SendBatteryInfo(CanProc_t* pHan);
static int32_t CanProc_GetCachedData(CanProc_t* pHan);
static void CanProc_UpdateSystemInfo(CanProc_t* pHan);
static void CanProc_SendDriveModeInfo(CanProc_t* pHan);
static void CanProc_SendCanStatusInfo(CanProc_t* pHan);
static int  CanProc_CalculateSpeedParams(CanProc_t* pHan);
static void CanProc_Process_UpdateSpeedInfo(CanProc_t* pHan);
static void CanProc_UpdateInfo(CanProc_t* pHan, uint64_t iOdoDiff);
static int  CanProc_ComposeRespCommnd(CanProc_t * pHan,uint8_t * pCmd, void * vpData);
static void CanProc_DataParse(CanProc_t * pHan, flexcan_frame_t* pMsg);
static void CanProc_SendFaultCodeInfo(CanProc_t* pHan,sFault_t * pFaults);
static void CanProc_Process_Fault(CanProc_t* pHan ,flexcan_frame_t* pFrame,int iId);
static void CanProc_Process_UpdateTempInfo(CanProc_t* pHan ,flexcan_frame_t* pFrame);
static void CanProc_Process_UpdateDriveMode(CanProc_t* pHan ,flexcan_frame_t* pFrame);
static void CanProc_Process_UpdateChargeInfo(CanProc_t* pHan ,flexcan_frame_t* pFrame);
static void  CanProc_Process_UpdateChargeTime(CanProc_t* pHan ,flexcan_frame_t* pFrame);
static void CanProc_Process_UpdateBatteryInfo(CanProc_t* pHan ,flexcan_frame_t* pFrame);
static void CanProc_PackSendInfo(uint32_t iOdoval, sTimeInfo_t  * pInfo, uint8_t *pdata);
static void CanProc_SendTextMsgInfo(CanProc_t* pHan, eAlertId_t iId, uint32_t iCode);
static int32_t CanProc_Process_SetRtc(CanProc_t * pHan,flexcan_frame_t* pFrame);
static int  CanProc_SendCommnd(CanProc_t* pHan,int32_t iCanId,uint32_t iLen,uint8_t *pdata,uint8_t mbIdx,uint8_t iFormat);
static int CanProc_ResetDistanceParams(CanProc_t* pHan);
/*----------------------------------------------------------------------------*
 * Function	    :		Cluster_Controller_Manager_CanProc_Init    	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_CanProc_Init(QueueHandle_t hAppQueueId)
{
	int32_t iRval = BCU_OK;
	BaseType_t iTaskRetval = 0;
	if(hCanProc.iIsInitialised == 0)
	{
		memset(&hCanProc, 0 ,sizeof(hCanProc));

		hCanProc.qHmiMsg = hAppQueueId;

		hCanProc.qRxMsg = xQueueCreate( CAN_MAX_QUEUE_MSG_COUNT,
										CAN_MSG_BUFFER_SIZE);
		hCanProc.qCmdMsg = xQueueCreate(1,8);

		if((hCanProc.qRxMsg != 0) && (hCanProc.qCmdMsg != 0))
		{
			uint16_t iStackSize =0;

			Cluster_Controller_Hal_Config_GetStackSize(&iStackSize);

			iTaskRetval = xTaskCreate(	CanProc_DataHandler,
										"CANP",
										iStackSize*2,
										&hCanProc,
										(configMAX_PRIORITIES - 2),
										&hCanProc.hRxThread);
			if (iTaskRetval == pdPASS)
			{
				iTaskRetval = xTaskCreate(CanProc_SendCommandHandler,
											"SCMD",
											iStackSize,
											&hCanProc,
											(configMAX_PRIORITIES - 2),
											&hCanProc.hCmdThread);

				if (iTaskRetval == pdPASS)
				{
					hCanProc.iIsInitialised = 1;
					CanProc_TimerConfigure();
				}
				else
				{
					iRval = BCU_NOK;
					Cluster_Controller_Manager_CanProc_DeInit();
					PRINTF("\r\nError: SendCommand: Thread create failed \r\n");
				}
			}
			else
			{
				iRval = BCU_NOK;
				Cluster_Controller_Manager_CanProc_DeInit();
				PRINTF("\r\nError: CanProc_DataHandler: Thread create failed \r\n");
			}
		}
		else
		{
			iRval = BCU_NOK;
			PRINTF("\r\nError:CanProc: xQueueCreate  failed \r\n");
		}
	}
	return iRval;
}
/*----------------------------------------------------------------------------*
 * Function	    :		Cluster_Controller_Manager_CanProc_DeInit	 		  *
 * Params	    :						       							      *
 *			    :   			         		                              *
 * Return value	:														  	  *
 * Description	:								                              *
 *----------------------------------------------------------------------------*/
void Cluster_Controller_Manager_CanProc_DeInit(void)
{
	if(hCanProc.iIsInitialised)
	{
		vTaskSuspend(hCanProc.hRxThread);
		CanProc_CleanUp(NULL);
	}
	memset(&hCanProc, 0, sizeof(CanProc_t));
}
/*----------------------------------------------------------------------------*
 * Function	    :		Cluster_Controller_Manager_CanProc_Stop		 		  *
 * Params	    :						       							      *
 *			    :   			         		                              *
 * Return value	:														  	  *
 * Description	:								                              *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_CanProc_Start(void)
{
	int32_t iRval = BCU_NOK;

	if((hCanProc.iIsInitialised) && (hCanProc.qRxMsg != NULL))
	{
		iRval = Cluster_Controller_Hal_SetReceiveQueueHandle(eModule_Can ,
															 hCanProc.qRxMsg);
	}
	return  iRval;
}
/*----------------------------------------------------------------------------*
 * Function	    :		Cluster_Controller_Manager_CanProc_Stop		 		  *
 * Params	    :						       							      *
 *			    :   			         		                              *
 * Return value	:														  	  *
 * Description	:								                              *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_CanProc_Stop(void)
{
	return 0;
}
/*----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Manager_CanProc_SendMessage	 		  *
 * Params	    :						       							      *
 *			    :   			         		                              *
 * Return value	:														  	  *
 * Description	:								                              *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_CanProc_SendMessage(sCanMessage_t *pMsg,uint8_t mbIdx)
{
	return Cluster_Controller_Hal_CanSendMessage(pMsg,mbIdx);
}
/*------------------------------------------------------------------------------*
 * Function	    :  		Cluster_Controller_Manager_CanProc_Reset				*
 * Params	    :		Void			                                      	*
 * Return value	:		Void		                                            *
 * Description	:																*
 *                                                                              *
 *------------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_CanProc_Reset(void)
{
	int32_t iRval = BCU_OK;
	if(hCanProc.iIsInitialised)
	{
		CanProc_Send_DummyData(&hCanProc);
	}
	return iRval;
}
/*------------------------------------------------------------------------------*
 * Function	    :  		Cluster_Controller_Manager_CanProc_ResetTrip			*
 * Params	    :		Void			                                      	*
 * Return value	:		Void		                                            *
 * Description	:																*
 *                                                                              *
 *------------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_CanProc_ResetTrip(void)
{
	int32_t iRval = BCU_OK;
	if(hCanProc.iIsInitialised)
	{
		CanProc_TripReset(&hCanProc);
	}
	return iRval;
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_ProductionReset		    		          	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_CanProc_ProductionReset(void)
{
	return CanProc_ProductionReset();
}
/*----------------------------------------------------------------------------*
 * Function	      	:	Cluster_Controller_Manager_CanProc_SetUdsQueueHan				          	  	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_CanProc_SetUdsQueueHan(QueueHandle_t qRxMsg)
{
	int32_t iRval = BCU_OK;
	if (hCanProc.iIsInitialised)
	{
		hCanProc.qUdsRxMsg = qRxMsg;
	}
	return iRval;
}
/*----------------------------------------------------------------------------*
 * Function	      	:	Cluster_Controller_Manager_CanProc_GetCanFrame    	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_CanProc_GetCanFrame(uint8_t * pFrame, uint8_t iFrameId)
{
	int32_t iRtval = BCU_NOK;
	(void)(pFrame);
	(void)(iFrameId);
//	memcpy(pFrame,&(hCanProc.sCanFrame[iFrameId-1][0]), sizeof(hCanProc.sCanFrame[iFrameId-1]));

	return iRtval;
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_TimerConfigure				          	  	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void CanProc_TimerConfigure()
{
	gpt_config_t gptConfig = {
								.clockSource = kGPT_ClockSource_Periph,
								.divider = 1,
								.enableRunInStop = false,
								.enableMode =true
							};

	GPT_Init(CANPROC_GPTIMER, &gptConfig);
	GPT_StopTimer(CANPROC_GPTIMER);
	GPT_StartTimer(CANPROC_GPTIMER); /*start timer*/
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_DataHandler						          	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void CanProc_DataHandler(void *arg)
{
	int32_t iDelay = READY_STATE_WAIT_TIME;
	int32_t iRval = BCU_NOK;
	CanProc_t *  pHan = (CanProc_t *)(arg);
	int iCount  = 0;
	sCanMsg_t sMsg;

	CanProc_ResetDistanceParams(pHan);

	xQueueReceive(pHan->qRxMsg,(void *)(&sMsg),(TickType_t)(iDelay));
	iRval  = Cluster_Controller_Hal_CanEnable();
    iDelay = MSG_RECEIVE_INIT_WAIT;

	if(iRval == BCU_OK)
	{
		while (1)
		{
			memset(&sMsg, 0, sizeof(sCanMsg_t));
			if(pdPASS == xQueueReceive(pHan->qRxMsg,(void *)(&sMsg),(TickType_t)(iDelay)))
			{
				if(sMsg.iEventId == 0)
				{
					if( pHan->bCanError == 1)
					{
						CanProc_SendTextMsgInfo(pHan,eAlert_CAN,eCAN_Working);
						pHan->bCanError = 0;
						CanProc_ResetDistanceParams(pHan);
					}
					iCount =0;
					CanProc_DataParse(pHan,&(sMsg.sRxFrame));
					if(pHan->bIsOdoCached == 0)
					{
						CanProc_Send_DummyData(pHan);
					}
					pHan->iSendCmdflag = 1;
					iDelay = MSG_RECEIVE_DEFAULT_WAIT;
				}
				else
				{
					pHan->iSendCmdflag = 0;
					Cluster_Controller_Hal_CanReset();
					iDelay = MSG_RECEIVE_ERROR_WAIT;
					if(pHan->bCanError == 0)
					{
						pHan->iCurTime = 0;
						pHan->iPrevTime = 0;
						
						CanProc_SendTextMsgInfo(pHan,eAlert_CAN,eCAN_Error);
					}
					pHan->bCanError = 1;
				}
			}
			else
			{
				pHan->iSendCmdflag = 0;
				if(iDelay != MSG_RECEIVE_ERROR_WAIT)
				{
					if(iCount == 0)
					{
						PRINTF("\r\nCAN : HMI Update with reset value \r\n");
						CanProc_Send_DummyData(pHan);
						pHan->bIsOdoCached = 1;
						iCount = 1;
					}

					CanProc_SendTextMsgInfo(pHan,eAlert_CAN,eCAN_NoData);
					pHan->bCanError = 1;
				}
				iDelay = MSG_RECEIVE_DEFAULT_WAIT;
			}
		}
	}
	else
	{
		CanProc_SendTextMsgInfo(pHan,eAlert_CAN,eCAN_NoData);
		pHan->bCanError = 1;
		PRINTF("\r\n Error: CanProc_DataHandler : Cluster_Controller_Hal_CanEnable Failed\r\n");
	}
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_CleanUp							          	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void CanProc_CleanUp(void* arg)
{
    (void)(arg);
	if(hCanProc.qRxMsg != 0)
	{
		vQueueDelete(hCanProc.qRxMsg);
	}
}

/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_ResetDistanceParams		          	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static int CanProc_ResetDistanceParams(CanProc_t* pHan)
{
	int iRetVal = BCU_OK;

	if(pHan->bIsResetDist == 0)
	{
		for (int32_t i = 0; i < SPEED_CACHE_SIZE; i++)
		{
			pHan->iSpeed[i] = 0;
			pHan->iTimeOffset[i] = 0;
		}
		pHan->iCurIndex = 0;
		pHan->iCurTime = 0;
		pHan->bIsResetDist = 1;
	}

	return iRetVal;
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_UpdateDistance		          	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static int CanProc_UpdateDistance(CanProc_t* pHan)
{
	int iRetVal = BCU_NOK;
	double iSum = 0;
	uint64_t TimeOffset = 0;
	uint64_t iTimeDiff =0;
	double iAvgSpeed =0;
	uint64_t iAvgTime  = 0;
	uint64_t iDistance = 0;

	uint64_t iMaxTime = 0xFFFFFFFF;

	if(pHan->iCurTime == 0)
	{
		CanProc_ResetDistanceParams(pHan);
		pHan->iCurTime = CanProc_GetCurrentTimestamp();
		pHan->iPrevTime = pHan->iCurTime;
	}
	else
	{
		pHan->iCurTime  = CanProc_GetCurrentTimestamp();

		if(pHan->iCurTime > pHan->iPrevTime)
		{
			iTimeDiff = (uint64_t)(pHan->iCurTime - pHan->iPrevTime);

		}
		else if(pHan->iCurTime < pHan->iPrevTime)
		{
			iTimeDiff =   (uint64_t)((iMaxTime - pHan->iPrevTime)+pHan->iCurTime);

			//PRINTF("\r\n Time Wrap: Cur : %d , iPrevTime: %d, Timediff  = %d\r\n",(uint32_t)pHan->iCurTime,(uint32_t)pHan->iPrevTime,(uint32_t)iTimeDiff);
		}
		else
		{
			iTimeDiff = 0;
		}

		if((iTimeDiff/24000) > 1000)
		{
			CanProc_ResetDistanceParams(pHan);
			pHan->iCurTime = CanProc_GetCurrentTimestamp();
			pHan->iPrevTime = pHan->iCurTime;
			PRINTF("\r\n Speed data timeout, Time Diff  = %d\r\n",(uint32_t)(iTimeDiff/24000));
		}
		else
		{
			pHan->bIsResetDist = 0;

			pHan->iTimeOffset[pHan->iCurIndex] = (iTimeDiff)/24000;

			pHan->iSpeed[pHan->iCurIndex] = (0.0121832 * pHan->sMotor.iRpm);//pHan->sMotor.iSpeed;

			pHan->iCurIndex++;
			pHan->iCurIndex = pHan->iCurIndex % SPEED_CACHE_SIZE;
			for (int32_t i = 0; i < SPEED_CACHE_SIZE; i++)
			{
				iSum = iSum + pHan->iSpeed[i];
				TimeOffset = TimeOffset + pHan->iTimeOffset[pHan->iCurIndex];
			}

			iAvgSpeed = (iSum / SPEED_CACHE_SIZE);
			iAvgTime  = (TimeOffset / SPEED_CACHE_SIZE);

			iDistance = (uint64_t)(round((iAvgSpeed * 1.0 * (double)iAvgTime)/36.0)); // distance in cm
			pHan->iCurOdo = (pHan->iCurOdo + iDistance);
			pHan->sMotor.iOdo = (uint32_t)(round(((double)pHan->iCurOdo*1.0)/100.0));  //iOdo is in meters
			pHan->iPrevTime = pHan->iCurTime;
		}
		iRetVal = BCU_OK;
	}
	return (iRetVal);
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_CalculateSpeedParams			          	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static int CanProc_CalculateSpeedParams(CanProc_t* pHan)
{
	int iRetVal = BCU_OK;

	pHan->sMotor.iSpeed = (uint32_t)round((0.0121832 * pHan->sMotor.iRpm)); //Xavion speed =(((RPM*100) * Wheel_diameter)/(gear_ratio *	axle_ratio))  g-8.1 a-8

	iRetVal = CanProc_UpdateDistance(pHan);

	if(iRetVal == BCU_OK)
	{
		if(pHan->iCurOdo >= pHan->iInitialOdo)
		{
			uint64_t iTripDiff  = (pHan->iCurOdo - pHan->iInitialOdo);
			pHan->sMotor.iTrip = (uint32_t)round((uint32_t)((pHan->iCurTrip + iTripDiff)/100));
			iRetVal = BCU_OK;
		}
		else
		{
			//PRINTF("\r\n ERROR: Initial Odo %d is greater than Current Odo %d :\r\n",pHan->iInitialOdo,pHan->iCurOdo);
		}
	}

	return iRetVal;
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_Send_DummyData					          	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void CanProc_Send_DummyData(CanProc_t* pHan)
{
	int iRtval = CanProc_GetCachedData(pHan);

	if(iRtval == BCU_OK)
	{
		CanProc_SendMotorInfo(pHan);
	}
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_GetCurrentTimestamp				          	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static uint32_t CanProc_GetCurrentTimestamp()
{
	uint32_t lmsec  =0;

	lmsec = GPT_GetCurrentTimerCount(CANPROC_GPTIMER);

    return (lmsec);
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_GetCachedData				          	  	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static int32_t CanProc_GetCachedData(CanProc_t* pHan)
{
	int iRval = BCU_NOK;

	if(pHan->bIsOdoCached == 0)
	{
		pHan->sMotor.iOdo   = 0;
		pHan->sMotor.iTrip  = 0;

		ClusterInfo_t sInfo = {0};
		Cluster_Controller_Hal_RtcUpdateDateTime();

		iRval = Cluster_Controller_GetSystemInfo(&sInfo);
		if(iRval == BCU_OK)
		{
			if((sInfo.iOdoVal != 0xFFFFFFFF)  && (sInfo.iCurTrip != 0xFFFFFFFF))
			{
				pHan->sMotor.iSpeed = 0 ;
				pHan->sMotor.iOdo   = sInfo.iOdoVal ;
				pHan->sMotor.iTrip  = sInfo.iCurTrip;

				pHan->iCurTrip 		= pHan->sMotor.iTrip*100; // in cm
				pHan->iCurOdo 		= pHan->sMotor.iOdo*100;
				pHan->iInitialOdo 	= pHan->sMotor.iOdo*100;
				pHan->iPrevOdo 		= pHan->sMotor.iOdo*100;
				pHan->iCurOdoDiff = 0;
				pHan->bIsOdoCached  = 1;
			}
			else
			{
				memset(&sInfo,0,sizeof(sInfo));
			}
		}
		else
		{
			PRINTF("\r\n Error : Failed to get Cached Data \r\n");
		}
	}
	else
	{
		pHan->sMotor.iSpeed = 0 ;
		pHan->iCurOdoDiff = 0;
		iRval = BCU_OK;
	}
	return iRval;
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_UpdateSystemInfo			          	  	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void CanProc_UpdateSystemInfo(CanProc_t* pHan)
{
	ClusterInfo_t sUpdateinfo = {0};

	sUpdateinfo.iOdoVal = pHan->sMotor.iOdo;
	sUpdateinfo.iCurTrip = pHan->sMotor.iTrip;

	Cluster_Controller_UpdateSystemInfo(&sUpdateinfo, eSpeedInfo);

	pHan->iPrevOdo = pHan->iCurOdo;
	pHan->iCurOdoDiff =0;
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_TripReset		    		          	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void CanProc_TripReset(CanProc_t* pHan)
{
	if(pHan->sMotor.iTrip != 0)
	{
		pHan->iCurTrip = 0;
		pHan->sMotor.iTrip = 0;

		pHan->sMotor.iOdo 	  = (uint32_t)pHan->iCurOdo/100;
		pHan->iInitialOdo 	  = pHan->iCurOdo;
		pHan->iPrevOdo 		  = pHan->iCurOdo;
		pHan->iCurOdoDiff 	  = 0;

		CanProc_SendMotorInfo(pHan);
		CanProc_UpdateSystemInfo(pHan);
	}
	CanProc_SendTextMsgInfo(pHan,eAlert_Trip,1);
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_ProductionReset		    		          	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static int32_t CanProc_ProductionReset(void)
{
	int32_t iRtval = BCU_NOK;

	ClusterInfo_t sUpdateinfo = {0};

	iRtval = Cluster_Controller_ResetSystemInfo(&sUpdateinfo);
	if(iRtval == BCU_NOK)
	{
		PRINTF("\r\n ERROR: CanProc_ProductionReset\r\n");
	}
	return iRtval;
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_SendMotorInfo    				          	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void CanProc_SendMotorInfo(CanProc_t* pHan)
{
	sMessage_t	  sMsg		= {0};

	sMsg.iEventId 			= eEvent_Motor;
	memcpy((char *)sMsg.sData,(char*)(&(pHan->sMotor)),sizeof(sMotorInfo_t));

	if((pHan->qHmiMsg != NULL) && (sMsg.iEventId >0))
	{
		xQueueSend(pHan->qHmiMsg, &sMsg, (TickType_t)(MSG_SEND_DEFAULT_WAIT));
	}
}

/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_SendBatteryInfo  				          	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void CanProc_SendBatteryInfo(CanProc_t* pHan)
{
	sMessage_t	  sMsg		= {0};
	sMsg.iEventId 			= eEvent_Battery;
	memcpy((char *)sMsg.sData,(char*)(&(pHan->sBattery)),sizeof(sBatteryInfo_t));

	if((pHan->qHmiMsg !=NULL) && (sMsg.iEventId >0))
	{
		xQueueSend(pHan->qHmiMsg, &sMsg, (TickType_t)(MSG_SEND_DEFAULT_WAIT));
	}
}

/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_ProcessSetRtc						          	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static int32_t CanProc_Process_SetRtc(CanProc_t * pHan, flexcan_frame_t* pFrame)
{
    (void)(pHan);
	int32_t iRval = BCU_OK;

    sTimeInfo_t sRTCInfo = {0};

    time_t timestamp = (pFrame->dataByte4 << 24) |
                       (pFrame->dataByte3 << 16) |
                       (pFrame->dataByte2 << 8) |
                       (pFrame->dataByte1);

    struct tm *timeinfo = gmtime(&timestamp);

//    PRINTF("Date & Time (UTC): %04d-%02d-%02d %02d:%02d:%02d\n",
//           timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
//           timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

    sRTCInfo.iMinute = (uint8_t)(timeinfo->tm_min + 30);
    sRTCInfo.iHour = (uint8_t)(timeinfo->tm_hour + 5);
    sRTCInfo.iDay = (uint8_t)timeinfo->tm_mday;
    sRTCInfo.iMonth = (uint8_t)timeinfo->tm_mon + 1;
    sRTCInfo.iYear = (uint8_t)((timeinfo->tm_year + 1900) - 2000);

    if (sRTCInfo.iMinute >= 60)
    {
        sRTCInfo.iMinute -= 60;    // Reset minutes to within 0-59
        sRTCInfo.iHour += 1;       // Add 1 to the hours
    }

    if (sRTCInfo.iHour >= 24) {
        sRTCInfo.iHour -= 24;  // Reset hours to within 0-23
        sRTCInfo.iDay += 1;    // Add 1 to the day
    }

    if ((sRTCInfo.iDay > 31) || (sRTCInfo.iMonth == 2 && sRTCInfo.iDay > 28) ||
        ((sRTCInfo.iMonth == 4 || sRTCInfo.iMonth == 6 || sRTCInfo.iMonth == 9 || sRTCInfo.iMonth == 11) && sRTCInfo.iDay > 30)) {

        sRTCInfo.iDay = 1;  // Reset to 1st of the next month
        sRTCInfo.iMonth += 1;

        if (sRTCInfo.iMonth > 12) {
            sRTCInfo.iMonth = 1;  // January
            sRTCInfo.iYear += 1;  // Increment the year
        }
    }

    if (sRTCInfo.iMonth == 2 && sRTCInfo.iDay > 29) {
        sRTCInfo.iDay = 1;
        sRTCInfo.iMonth += 1;  // Move to March
    }

    sRTCInfo.iMonth = (uint8_t)((sRTCInfo.iMonth > 12) ? 12 : sRTCInfo.iMonth);

//    PRINTF("\r\n **The RTC frame: [%x %x %x %x %x %x]",
//           sRTCInfo.iSecond, sRTCInfo.iMinute, sRTCInfo.iHour,
//           sRTCInfo.iDay, sRTCInfo.iMonth, sRTCInfo.iYear);

    Cluster_Controller_Manager_RtcProc_CheckForValidData(&sRTCInfo);

    iRval = Cluster_Controller_Hal_RtcSetDateTime(&sRTCInfo);

    return iRval;
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_SendTextMsgInfo  				          	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void CanProc_SendTextMsgInfo(CanProc_t* pHan, eAlertId_t iId, uint32_t iCode)
{
	sMessage_t	  sMsg		= {0};
	sAlertInfo_t sAlertInfo = {0};

	if(pHan->qHmiMsg != NULL)
	{
		sMsg.iEventId 		  = eEvent_Alert;

		sAlertInfo.iId   = iId;
		sAlertInfo.iCode = iCode;
		memcpy((char *)sMsg.sData,(char*)(&(sAlertInfo)),sizeof(sAlertInfo_t));

		xQueueSend(pHan->qHmiMsg, &sMsg, (TickType_t)(MSG_SEND_DEFAULT_WAIT));
	}
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_DataParse						          	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void CanProc_Process_SendTeleTailInfo(CanProc_t * pHan)
{
	int32_t iRval = BCU_NOK;
	uint8_t sData[8] = {0};
	iRval = Cluster_Controller_SwitchProc_GetHwStatus(&(pHan->sHwStatus));

	if(iRval == BCU_OK)
	{
		sData[0] = (uint8_t)((( pHan->sHwStatus.bInput8 & 0x01) << 7) |
					         (( pHan->sHwStatus.bInput7 & 0x01) << 6) |
							 ((pHan->sHwStatus.bInput6 & 0x01) << 5) |
							 ((pHan->sHwStatus.bInput5 & 0x01) << 4)|
							 ((pHan->sHwStatus.bInput4 & 0x01) << 3) |
							 ((pHan->sHwStatus.bInput3 & 0x01) << 2)|
							 ((pHan->sHwStatus.bInput2 & 0x01) << 1) |
							  (pHan->sHwStatus.bInput1 & 0x01));

		sData[1] = (uint8_t)(((pHan->sHwStatus.bInput10 & 0x01) << 1) |
				              (pHan->sHwStatus.bInput9 & 0x01));

		iRval = CanProc_SendCommnd(pHan,eCANID_Internal,8,&sData[0],CAN_SEND_MBID,1);
		if(iRval != BCU_OK)
		{
			PRINTF("\r\nError: CAN_PROC HARDWARE_STATUS CAN Send failed\r\n");
		}
	}
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_Process_SendVersionInfo						          	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void CanProc_Process_SendVersionInfo(CanProc_t * pHan)
{
	(void)(pHan);
	int32_t iRval = BCU_NOK;

	uint8_t sData[8] = {0};
	uint16_t iSwVersion = 0;
	uint16_t iHwVersion = 0;

	Cluster_Controller_Hal_ConfigGetSwHwVersion(&iSwVersion, &iHwVersion);

	sData[0] = (uint8_t)(iSwVersion & 0xFF);
	sData[1] = (uint8_t)((iSwVersion >>8) & 0xFF);
	sData[2] = (uint8_t)(iHwVersion & 0xFF);
	sData[3] = (uint8_t)((iHwVersion >>8) & 0xFF);


	iRval = CanProc_SendCommnd(pHan,eCANID_Frame11,8,&sData[0],CAN_SEND_MBID,1);
	if(iRval != BCU_OK)
	{
		PRINTF("\r\nError: CAN_PROC READ_SWHW_VERSION CAN Send failed\r\n");
	}
}

/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_Process_SetBackLight						          	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void CanProc_Process_SetBackLight(CanProc_t * pHan)
{
	(void)(pHan);
#if ENABLE_PWM_CONTROL
	uint32_t iRval = BCU_NOK;
	ClusterInfo_t sUpdateinfo = {0};

	sUpdateinfo.iBrightness = pFrame->dataByte1;

	iRval = Cluster_Controller_UpdateSystemInfo(&sUpdateinfo, eBrightnessInfo);
	if(iRval == BCU_OK)
	{
		Cluster_Controller_Hal_SetBrightness(sUpdateinfo.iBrightness);
	}
#endif
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_Process_FactoryReset						          	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void CanProc_Process_FactoryReset(CanProc_t * pHan)
{
	int32_t iRtval = BCU_NOK;

	iRtval = CanProc_ProductionReset();
	if(iRtval == BCU_OK)
	{
		pHan->sMotor.iTrip = 0;
		pHan->sMotor.iOdo  = 0;
		pHan->iInitialOdo  = 0;
		pHan->iPrevOdo 	   = 0;
		pHan->iCurOdo 	   = 0;
		pHan->iCurTrip 	   = 0;
		CanProc_ResetDistanceParams(pHan);
		CanProc_SendMotorInfo(pHan);
	}
}
/*----------------------------------------------------------------------------*
  Function	      	:	CanProc_SendCommnd
  Params	    	:
  Return value		:
  Description		:
 *----------------------------------------------------------------------------*/
static int  CanProc_SendCommnd(CanProc_t* pHan ,int32_t iCanId,uint32_t iLen,uint8_t *pdata,uint8_t mbIdx,uint8_t iFormat)
{
	int iRetVal = BCU_NOK;
	sCanMessage_t CanMsg;

	CanMsg.iCanId = iCanId;
	CanMsg.iFormat = iFormat;
	CanMsg.iDataLen = iLen;

	memcpy(CanMsg.sData,pdata,sizeof(CanMsg.sData));

	if(pHan->iSendCmdflag == 1)
	{
		iRetVal = Cluster_Controller_Hal_CanSendMessage(&CanMsg,mbIdx);
	}
	return iRetVal;
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_UpdateInfo					          	  	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void CanProc_UpdateInfo(CanProc_t* pHan, uint64_t iOdoDiff)
{
	if ((pHan->sMotor.iSpeed < 3) && (iOdoDiff > 75))
	{
		CanProc_UpdateSystemInfo(pHan);
		pHan->iCurTime = 0;
		pHan->iPrevTime = 0;
	}
	else if((pHan->sMotor.iSpeed <= 10) && (iOdoDiff > 1000))
	{
		CanProc_UpdateSystemInfo(pHan);
	}
	else if((pHan->sMotor.iSpeed > 10) && (pHan->sMotor.iSpeed <= 30) && (iOdoDiff > 3000))
	{
		CanProc_UpdateSystemInfo(pHan);
	}
	else if((pHan->sMotor.iSpeed > 30) && (pHan->sMotor.iSpeed <= 50) && (iOdoDiff > 5000))
	{
		CanProc_UpdateSystemInfo(pHan);
	}
	else if((pHan->sMotor.iSpeed > 50) && (iOdoDiff > 8000))
	{
		CanProc_UpdateSystemInfo(pHan);
	}
}
/*----------------------------------------------------------------------------*
  Function	      	:	UDSProc_ComposeCommnd
  Params	    	:
  Return value		:
  Description		:
 *----------------------------------------------------------------------------*/
static int  CanProc_ComposeRespCommnd(CanProc_t* pHan ,uint8_t * pCmd, void * vpData)
{
	int32_t iRval = BCU_NOK;

	uint8_t i = 2;
	uint8_t k = 0;

	if(pCmd[0] != SLN_UMBER)
	{
		char * pData = (char *)vpData;

		for(uint8_t j = 0; j < 12; j++)
		{
			pCmd[i] = pData[j];

			i += 1;

			if(i == 8)
			{
				i = 2;

				k += 1;

				if(k == 2)
				{
					pCmd[1] = VINFO_READ_END;
				}

				for (uint8_t m = 0; m < 8; m++)
				{
					PRINTF("%02X ", pCmd[m]);
				}

				iRval = CanProc_SendCommnd(pHan,eCANID_Internal,8,&pCmd[0],CAN_SEND_MBID,1);

				if(iRval != BCU_OK)
				{
					PRINTF("\r\nError: CAN_PROC READ_SWHW_VERSION CAN Send failed\r\n");
				}
			}
		}
	}
	else
	{
		uint32_t * pData = (uint32_t *)vpData;

		pCmd[2] = (uint8_t)((*pData >> 24) & 0xFF);
		pCmd[3] = (uint8_t)((*pData >> 16) & 0xFF);
		pCmd[4] = (uint8_t)((*pData >> 8) & 0xFF);
		pCmd[5] = (uint8_t)(*pData & 0xFF);

		for (uint8_t m = 0; m < 8; m++)
		{
			PRINTF("%02X ", pCmd[m]);
		}

		iRval = CanProc_SendCommnd(pHan,eCANID_Internal,8,&pCmd[0],CAN_SEND_MBID,1);

		if(iRval != BCU_OK)
		{
			PRINTF("\r\nError: CAN_PROC READ_SWHW_VERSION CAN Send failed\r\n");
		}
	}
	return iRval;
}
//
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_DataParse						          	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void CanProc_DataParse(CanProc_t * pHan,flexcan_frame_t* pFrame)
{
	int iCanId = 0;
	int iLen   = 0;
	if (pFrame != NULL)
	{
		if(pFrame->format == kFLEXCAN_FrameFormatStandard)
		{
			iCanId = pFrame->id >> CAN_ID_STD_SHIFT;
		}
		else
		{
			iCanId = pFrame->id >> CAN_ID_EXT_SHIFT;
		}
		iLen   = pFrame->length;
		switch(iCanId)
		{
			case eCANID_Frame1:
			{
				if(iLen == 8)
				{
					if(pHan->bIsOdoCached == 0)
					{
						CanProc_Send_DummyData(pHan);
					}

					pHan->sMotor.iRpm   = (pFrame->dataByte0 * 100);

					if(pHan->sMotor.iRpm < 15000)
					{
						pHan->sMotor.iRange = pFrame->dataByte1;

						// for power, factor is 4
						int16_t  iPower = (int16_t)(((pFrame->dataByte3 << 8) | (pFrame->dataByte2)));

						pHan->sMotor.bRegen = (iPower & 0x8000)? 1:0;

						pHan->sMotor.iPower = (uint32_t)((abs(iPower)) * 4);
					
						CanProc_CalculateSpeedParams(pHan);

						CanProc_Process_UpdateSpeedInfo(pHan);

						CanProc_Process_UpdateDriveMode(pHan,pFrame);

						CanProc_Process_UpdateChargeInfo(pHan,pFrame);
					}
				}
			}
			break;
			case eCANID_Frame2:
			{
				if(iLen == 8)
				{
					CanProc_Process_UpdateBatteryInfo(pHan, pFrame);
				}
			}		
			break;
			case eCANID_Frame3:
			{
				if(iLen == 8)
				{
					CanProc_Process_UpdateTempInfo(pHan, pFrame);
				}
			}
			break;
			case eCANID_Frame5:
			case eCANID_Frame6:
			case eCANID_Frame7:
			case eCANID_Frame8:
			case eCANID_Frame9:
			{
				if(iLen == 8)
				{
					CanProc_Process_Fault(pHan,pFrame,iCanId);
				}
			}
			break;
			case eCANID_Frame10:
			{
				if(iLen == 8)
				{
					CanProc_Process_UpdateChargeTime(pHan,pFrame);
				}
			}
			break;
			case eCANID_Internal:
			{
				if(iLen == 8)
				{
					switch(pFrame->dataByte0)
					{
						case FLASH_MODE:
						{
							Cluster_Controller_Hal_FirmwareDownload_Init();
						}
						break;
						case RTC_SET_TIME:
						{
							CanProc_Process_SetRtc(pHan, pFrame);
							Cluster_Controller_Hal_RtcUpdateDateTime();
						}
						break;
						case FACTORY_RESET:
						{
							CanProc_Process_FactoryReset(pHan);
						}
						break;
						case BACKLIGHT_SET:
						{
							CanProc_Process_SetBackLight(pHan);
						}
						break;
						case HARDWARE_STATUS:
						{
							CanProc_Process_SendTeleTailInfo(pHan);
						}
						break;
						case READ_SWHW_VERSION:
						{
							CanProc_Process_SendVersionInfo(pHan);
						}
						break;
						case VINFO_WRITE_START:
						{
							Cluster_Controller_GetVehicleSpecInfo(&(pHan->sSysInfoCache));
							pHan->sInfoCacheData[0] = pFrame->dataByte2;
							pHan->sInfoCacheData[1] = pFrame->dataByte3;
							pHan->sInfoCacheData[2] = pFrame->dataByte4;
							pHan->sInfoCacheData[3] = pFrame->dataByte5;
							pHan->sInfoCacheData[4] = pFrame->dataByte6;
							pHan->sInfoCacheData[5] = pFrame->dataByte7;
						}
						break;
						case VINFO_WRITE_CONTINUE:
						case VINFO_WRITE_END:
						{
							pHan->sInfoCacheData[6] = pFrame->dataByte2;
							pHan->sInfoCacheData[7] = pFrame->dataByte3;
							pHan->sInfoCacheData[8] = pFrame->dataByte4;
							pHan->sInfoCacheData[9] = pFrame->dataByte5;
							pHan->sInfoCacheData[10] = pFrame->dataByte6;
							pHan->sInfoCacheData[11] = pFrame->dataByte7;

							memcpy(&(pHan->sSysInfo),&(pHan->sSysInfoCache),sizeof(pHan->sSysInfo));

							if(pFrame->dataByte1 & SLN_UMBER) //SlNum.
							{
								pHan->sSysInfo.iSlNum = ((uint32_t)pHan->sInfoCacheData[3] << 24) |
												 ((uint32_t)pHan->sInfoCacheData[2] << 16) |
												 ((uint32_t)pHan->sInfoCacheData[1] << 8)  |
												 (uint32_t)pHan->sInfoCacheData[0];
								pHan->sSysInfoCache.iSlNum = pHan->sSysInfo.iSlNum;
							}
							else if(pFrame->dataByte1 & VIN) //VIN.
							{
								memcpy(pHan->sSysInfo.sVin, pHan->sInfoCacheData, sizeof(pHan->sSysInfo.sVin));

								memcpy(pHan->sSysInfoCache.sVin,pHan->sSysInfo.sVin, sizeof(pHan->sSysInfo.sVin));
							}

							Cluster_Controller_SetVehicleSpecInfo(&(pHan->sSysInfo), eProductInfo);

							memset(&(pHan->sSysInfo), 0 , sizeof(pHan->sSysInfo));
							memset(pHan->sInfoCacheData, 0 , sizeof(pHan->sInfoCacheData));
						}
						break;
						case VINFO_READ:
						{
							uint8_t sData[8] = {0};

							sDeviceInfo_t sInfo = {0};

							Cluster_Controller_GetVehicleSpecInfo(&sInfo);

							if((sInfo.iSlNum == 0xFFFFFFFF) && ((unsigned char) sInfo.sVin[0] == 0xFF))
							{
								DbgConsole_Printf("\r\nThe Device is New Set the Device details\r\n");
							}
							else
							{
								if((pFrame->dataByte1 & SLN_UMBER) == SLN_UMBER)
								{
									sData[0] = SLN_UMBER;
									sData[1] = VINFO_READ_END;

									CanProc_ComposeRespCommnd(pHan,sData,&sInfo.iSlNum);
								}
								else if((pFrame->dataByte1 & VIN) == VIN)
								{
									sData[0] = VIN;
									sData[1] = VINFO_READ_CONTINUE;

							CanProc_ComposeRespCommnd(pHan,sData,sInfo.sVin);
						}
					}
				}
				break;
				case SET_ODO:
				{
					pHan->sMotor.iOdo = (uint32_t)(
							((uint32_t)(pFrame->dataByte4) << 24) |
							((uint32_t)(pFrame->dataByte3) << 16) |
							((uint32_t)(pFrame->dataByte2) << 8) |
							(uint32_t)(pFrame->dataByte1)
					);

					CanProc_UpdateSystemInfo(pHan);
					CanProc_SendMotorInfo(pHan);
				}
				break;
				default:
					break;
				}
			}
		}
		break;

#if(UDS_SERVICE_ENABLE == 1)
			case eCANID_UDSRx:
			{
				if (pHan->qUdsRxMsg != NULL)
				{
					if (xQueueSend(pHan->qUdsRxMsg, pFrame, (TickType_t)(MSG_SEND_DEFAULT_WAIT)) != pdPASS)
					{
						PRINTF("\r\nFailed to send message to the queue\r\n");
					}
				}
			}
			break;
#endif
			default:
				PRINTF("\r\n DataParse Default CAN ID: 0x%x\r\n",iCanId);
			break;
		}
	}
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_Process_UpdateSpeedInfo				          *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void CanProc_Process_UpdateSpeedInfo(CanProc_t* pHan)
{
	uint64_t iOdoDiff = (uint64_t)((pHan->iCurOdo - pHan->iPrevOdo)/100);

	if((pHan->iCurSpeed != pHan->sMotor.iSpeed) ||
			(pHan->bIsSpeedUpdate == 0) ||
			(pHan->iCurRpm != pHan->sMotor.iRpm) ||
			(pHan->iCurPower != pHan->sMotor.iPower) ||
			(pHan->iCurRange != pHan->sMotor.iRange) ||
			((iOdoDiff - pHan->iCurOdoDiff) > 100))
	{
		pHan->iCurSpeed  = pHan->sMotor.iSpeed;
		pHan->iCurRpm    = pHan->sMotor.iRpm;
		pHan->iCurPower  = pHan->sMotor.iPower;
		pHan->iCurRange  = pHan->sMotor.iRange;
		pHan->iCurOdoDiff	 = iOdoDiff;
		pHan->bIsSpeedUpdate = 1;
		CanProc_SendMotorInfo(pHan);
	}

	CanProc_UpdateInfo(pHan,iOdoDiff);
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_Process_UpdateDriveMode				          *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void CanProc_Process_UpdateDriveMode(CanProc_t* pHan ,flexcan_frame_t* pFrame)
{
	int32_t iVal = pFrame->dataByte4 & 0x03;   //byte4 bit0-bit1 gear mode

	pHan->sDriveMode.iDriveVal =  (pFrame->dataByte4 & 0x04)? 1:0; //byte4 bit2  drive mode

	if( (pHan->sDriveMode.iDriveVal != pHan->iDriveMode) || (pHan->iGearMode != iVal))
	{
		if((iVal & 0x01) != 0)
		{
			pHan->sDriveMode.iGearVal = eGearmode_Drive;	    //Drive
		}
		else if((iVal & 0x02) != 0)
		{
			pHan->sDriveMode.iGearVal =  eGearmode_Reverse;     //Reverse
		}
		else
		{
			pHan->sDriveMode.iGearVal =  eGearmode_Neutral; 	//Neutral
		}

		pHan->iGearMode = iVal	;
		pHan->iDriveMode = pHan->sDriveMode.iDriveVal;
		CanProc_SendDriveModeInfo(pHan);
	}
}

/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_SendFaultCodeInfo  				          	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void CanProc_SendFaultCodeInfo(CanProc_t* pHan, sFault_t * pFaults)
{
	sMessage_t	  sMsg		= {0};

	if(pHan->qHmiMsg != NULL)
	{
		sMsg.iEventId 			= eEvent_Fault;
		memcpy((char *)sMsg.sData,(char*)(pFaults),sizeof(sFault_t));
		xQueueSend(pHan->qHmiMsg, &sMsg,(TickType_t)(MSG_SEND_DEFAULT_WAIT));
	}
	pHan->iFaultFlag = 1;
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_SendDriveModeInfo				          	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void CanProc_SendDriveModeInfo(CanProc_t* pHan)
{
	sMessage_t	  sMsg		= {0};
	if(pHan->qHmiMsg != NULL)
	{
		sMsg.iEventId 			= eEvent_DriveMode;
		memcpy((char *)sMsg.sData,(char*)(&(pHan->sDriveMode)),sizeof(sDriveMode_t));
		xQueueSend(pHan->qHmiMsg, &sMsg, (TickType_t)(MSG_SEND_DEFAULT_WAIT));
	}
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_SendChargingInfo			          	  	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void CanProc_SendChargingInfo(CanProc_t* pHan)
{
	sMessage_t	  sMsg		= {0};

	if(pHan->qHmiMsg != NULL)
	{
		sMsg.iEventId 			= eEvent_Charge;
		memcpy((char *)sMsg.sData,(char*)(&(pHan->sCharge)),sizeof(sChargeInfo_t));
		xQueueSend(pHan->qHmiMsg, &sMsg, (TickType_t)(MSG_SEND_DEFAULT_WAIT));
	}
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_SendCanStatusInfo				          	  	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void CanProc_SendCanStatusInfo(CanProc_t* pHan)
{
	sMessage_t	  sMsg		= {0};

	if(pHan->qHmiMsg != NULL)
	{
		sMsg.iEventId 			= eEvent_Indicator;

		memcpy((char *)sMsg.sData,(char*)(&(pHan->sCanStatus)),sizeof(sStatus_t));

		xQueueSend(pHan->qHmiMsg, &sMsg, (TickType_t)(MSG_SEND_DEFAULT_WAIT));
	}
}

/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_Process_UpdateBatteryInfo				          *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void CanProc_Process_UpdateBatteryInfo(CanProc_t* pHan ,flexcan_frame_t* pFrame)
{
	uint32_t iBatteryFault     = 0;
	uint32_t iPTStatus = 0;
	uint32_t iSoc    		   = pFrame->dataByte0;
	uint32_t iVoltage          = (uint32_t)((pFrame->dataByte1 << 8) |	(pFrame->dataByte2));
	uint32_t iCurrent          = pFrame->dataByte3;

	if(iSoc > 100)
	{
		iSoc = 100;
	}

	if((pHan->sBattery.iSoc != iSoc) ||
			(pHan->iSOCUpdateCount >100))
	{
		pHan->sBattery.iSoc = iSoc;
		pHan->sBattery.iVoltage = iVoltage;
		pHan->sBattery.iCurrent = iCurrent;

		CanProc_SendBatteryInfo(pHan);
		pHan->iSOCUpdateCount = 0;
	}
	else
	{
		pHan->iSOCUpdateCount = pHan->iSOCUpdateCount + 1;
	}

	iBatteryFault = (pFrame->dataByte4 & 0x03);

	iPTStatus =  pFrame->dataByte5 & 0xE0;

	if((pHan->iPrevBatteryFault != iBatteryFault) ||
			(pHan->iPrevPTStatus != iPTStatus))
	{
		pHan->iPrevBatteryFault = iBatteryFault;
		pHan->iPrevPTStatus = iPTStatus;

		pHan->sCanStatus.bBatMalFunc   = (iBatteryFault & 0x01);
		pHan->sCanStatus.bBatFault     = (iBatteryFault & 0x02);

		pHan->sCanStatus.bPTMalFunc = (iPTStatus & 0x20);
		pHan->sCanStatus.bPTFault   = (iPTStatus & 0x40);
		pHan->sCanStatus.bHvil      = (iPTStatus & 0x80);

		CanProc_SendCanStatusInfo(pHan);
	}
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_Process_UpdateTempInfo				          *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void CanProc_Process_UpdateTempInfo(CanProc_t* pHan ,
											flexcan_frame_t* pFrame)
{
	pHan->sBattery.iTemp  	= pFrame->dataByte0;

	if(pHan->iCurBatTemp != pHan->sBattery.iTemp)
	{
		pHan->iCurBatTemp = pHan->sBattery.iTemp;
		CanProc_SendBatteryInfo(pHan);
	}

	pHan->sCanStatus.bCoolant 	=  (pFrame->dataByte4 & 0x01);

	if(pHan->bPrevCoolant != pHan->sCanStatus.bCoolant)
	{
		pHan->bPrevCoolant =  pHan->sCanStatus.bCoolant;

		CanProc_SendCanStatusInfo(pHan);
	}
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_Process_UpdateChargeInfo				  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void  CanProc_Process_UpdateChargeInfo(CanProc_t* pHan ,flexcan_frame_t* pFrame)
{
	uint32_t iValue = pFrame->dataByte5 & 0x07;

	if(pHan->iChargerStatus != iValue)
	{
		pHan->sCharge.bType    = (iValue & 0x01);
		pHan->sCharge.bGunlock = (iValue & 0x02);
		pHan->sCharge.bStatus  = (iValue & 0x04);
		pHan->iChargerStatus   = iValue;
		CanProc_SendChargingInfo(pHan);
	}
}

/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_Process_UpdateChargeInfo				  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void  CanProc_Process_UpdateChargeTime(CanProc_t* pHan ,flexcan_frame_t* pFrame)
{
	uint32_t iValue = (uint32_t)((pFrame->dataByte0 << 8)| pFrame->dataByte1);

	if(pHan->iChargeTime != iValue)
	{
		pHan->iChargeTime = iValue;
		pHan->sCharge.iTime    = iValue/10;
		CanProc_SendChargingInfo(pHan);
	}
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_Process_Fault				  				  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void  CanProc_Process_Fault(CanProc_t* pHan ,flexcan_frame_t* pFrame,int iId)
{
	uint16_t iFaultId = 0;

	switch(iId)
	{
		case eCANID_Frame5:
			iFaultId = 0;
		break;
		case eCANID_Frame6:
			iFaultId = 1;
		break;
		case eCANID_Frame7:
			iFaultId = 2;
		break;
		case eCANID_Frame8:
			iFaultId = 3;
		break;
		case eCANID_Frame9:
			iFaultId = 4;
		break;
		default:
			iFaultId = 0xFF;
		break;
	}

	if((iFaultId < 5))
	{
		sFault_t			sFault = {0};

		sFault.iId = iFaultId;
		sFault.ifault0 = (uint16_t)(((pFrame->dataByte1 << 8) | pFrame->dataByte0) & 0xFFFF);
		sFault.ifault1 = (uint16_t)(((pFrame->dataByte3 << 8) | pFrame->dataByte2) & 0xFFFF);
		sFault.ifault2 = (uint16_t)(((pFrame->dataByte5 << 8) | pFrame->dataByte4) & 0xFFFF);
		sFault.ifault3 = (uint16_t)(((pFrame->dataByte7 << 8) | pFrame->dataByte6) & 0xFFFF);

		if( (pHan->sFault[iFaultId].ifault0 != sFault.ifault0 ) ||
				(pHan->sFault[iFaultId].ifault1 != sFault.ifault1 )||
				(pHan->sFault[iFaultId].ifault2 != sFault.ifault2 )||
				(pHan->sFault[iFaultId].ifault3 != sFault.ifault3 ) ||
				(pHan->sFault[iFaultId].icount > 50))
		{
			CanProc_SendFaultCodeInfo(pHan, &(sFault));
			pHan->sFault[iFaultId].ifault0 = sFault.ifault0;
			pHan->sFault[iFaultId].ifault1 = sFault.ifault1 ;
			pHan->sFault[iFaultId].ifault2 = sFault.ifault2 ;
			pHan->sFault[iFaultId].ifault3 = sFault.ifault3 ;
			pHan->sFault[iFaultId].icount = 0;
		}
		else
		{
			pHan->sFault[iFaultId].icount = pHan->sFault[iFaultId].icount+1;
		}
	}
}
/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_SendCommandHandler						          	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void CanProc_SendCommandHandler(void *arg)
{
	CanProc_t *  pHan = (CanProc_t *)(arg);
	uint8_t sData[8] = {0};
	sTimeInfo_t sRtcInfo = {0};
	flexcan_frame_t rxFrame;
	uint8_t iSwVerSendFlag = 0;
	int iDelay = 5000;
	int iRetval = BCU_OK;

	while (1)
	{
		memset(&rxFrame, 0, sizeof(flexcan_frame_t));

		xQueueReceive(pHan->qCmdMsg,(void *)(&rxFrame),(TickType_t)(iDelay));

		if(pHan->iSendCmdflag == 1)
		{
			memset(&sRtcInfo, 0, sizeof(sTimeInfo_t));

			Cluster_Controller_Hal_RtcGetDateTime(&sRtcInfo);

			CanProc_PackSendInfo((pHan->sMotor.iOdo/1000),&sRtcInfo,&sData[0]);

			iRetval = CanProc_SendCommnd(pHan,eCANID_Frame4,8,&sData[0],CAN_SEND_MBID,1);
			if(iRetval == BCU_OK)
			{
				iDelay = 1000;
			}
			else
			{
				PRINTF("\r\n CAN Module Error,Re-check after 5 seconds\r\n");
				iDelay = 5000;
			}

			if(iSwVerSendFlag == 0)
			{
				iSwVerSendFlag = 1;
				for(int i = 0; i < 5; i++)
				{
					CanProc_Process_SendVersionInfo(pHan);
					xQueueReceive(pHan->qCmdMsg,(void *)(&rxFrame),(TickType_t)(200));
				}
			}
		}
		else
		{
			iDelay = 5000;
		}
	}
}

/*----------------------------------------------------------------------------*
 * Function	      	:	CanProc_PackSendInfo				          	  	  *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void CanProc_PackSendInfo(uint32_t iOdoval, sTimeInfo_t  * pInfo, uint8_t *pdata)
{
	struct tm timeinfo = {0};

	timeinfo.tm_year = (2000 + pInfo->iYear) - 1900; // Year since 1900
	timeinfo.tm_mon  = pInfo->iMonth - 1;             // Month (0-11, so January is 0)
	timeinfo.tm_mday = pInfo->iDay;                  // Day of the month (1-31)

	if (pInfo->bPM)
	{
		if (pInfo->iHour != 12)
		{
			timeinfo.tm_hour = pInfo->iHour + 12;    // Convert PM hour to 24-hour format
		}
		else
		{
			timeinfo.tm_hour = pInfo->iHour;         // 12 PM is 12 in 24-hour format
		}
	}
	else
	{
		if (pInfo->iHour == 12)
		{
			timeinfo.tm_hour = 0;                    // 12 AM is 0 in 24-hour format
		}
		else
		{
			timeinfo.tm_hour = pInfo->iHour;         // AM hours remain unchanged
		}
	}

	timeinfo.tm_min = pInfo->iMinute;                // Minutes (0-59)
	timeinfo.tm_sec = 0;                             // Seconds (0-59)
	timeinfo.tm_isdst = -1;                          // Let mktime determine DST

	time_t timestamp = mktime(&timeinfo);

	pdata[0] = (uint8_t)((timestamp >> 0) & 0xFF);
	pdata[1] = (uint8_t)((timestamp >> 8) & 0xFF);
	pdata[2] = (uint8_t)((timestamp >> 16) & 0xFF);
	pdata[3] = (uint8_t)((timestamp >> 24) & 0xFF);

	pdata[4] = (uint8_t)((iOdoval) & 0xFF);
	pdata[5] = (uint8_t)((iOdoval >> 8) & 0xFF);       // Lower byte of the odometer value
	pdata[6] = (uint8_t)((iOdoval >> 16) & 0xFF);
	pdata[7] = (uint8_t)((iOdoval >> 24) & 0xFF); // Upper byte of the odometer value
}
