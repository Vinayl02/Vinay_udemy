/*-----------------------------------------------------------------------------*
 *	OptM Media Solutions Confidential										   *
 *	Copyright (C) OptM Media Solutions Pvt Ltd - All Rights Reserved.	   	   *
 *  Dissemination of this information or reproduction or redistribution of 	   *
 *  this material is  strictly forbidden unless prior written permission is	   *
 *  obtained from OptM Media Solutions Pvt Ltd								   *
 *	File Name	:	cluster_controller_manager_rtcproc.c 					   *
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
#include "fsl_gpt.h"
#include "fsl_debug_console.h"
#include "cluster_common.h"
#include "cluster_controller_hal.h"
#include "cluster_controller_hal_config.h"
#include "cluster_controller_manager_rtcproc.h"
#include "cluster_controller_manager_canproc.h"
#include "cluster_controller_manager_switchproc.h"
#include "cluster_controller_manager_systeminfo.h"
/*----------------------------------------------------------------------------*
* Macro ,Structure and Enum definitions                                       *
*-----------------------------------------------------------------------------*/
#define MAX_QUEUE_MSG_COUNT				10
#define MSG_BUFFER_SIZE					sizeof(sMessage_t)
#define MSG_QUEUE_BUFFER_SIZE 			MSG_BUFFER_SIZE + 16
#define RTC_SET_MODE_COUNT              10
#define RTC_SET_OPTION_COUNT            2
#define RTC_SAVE_AND_EXIT               4
#define RTC_RECEIVE_DELAY               25000 //25 sec
#define RTC_SAVE_AND_EXIT_DELAY         10000 //10 Seconds

typedef struct
{
	bool         	bIsPM;
	bool			iIsSetMode;
	int32_t     	iIsInitialised;
	int32_t			iModeCount;
	int32_t         iIsAMorPM;
	uint8_t         iPrevKey;
	uint32_t        iPrevTime;
	uint32_t        iCurTime;
	eRtcState_t 	iState;
	TaskHandle_t    hRTCThread;
	QueueHandle_t   qRxMsg;
	QueueHandle_t  	qHmiMsg;
}RtcProc_t;
/*----------------------------------------------------------------------------*
 * Static and global variable definition	                         		  *
 *----------------------------------------------------------------------------*/
static RtcProc_t hRtcProc = {0};

static sTimeInfo_t sRtcInfo = {0};
/*----------------------------------------------------------------------------*
 * Local function definition	                         				      *
 *----------------------------------------------------------------------------*/
static void RtcProc_ModeHandler(void *arg);
static void RtcProc_GetCurrentInfo(const RtcProc_t * pHan);
static void RtcProc_Set_Initialise(RtcProc_t * pHan);
static void RtcProc_Set_UpdateState(RtcProc_t * pHan);
static void RtcProc_Set_SaveChanges(RtcProc_t * pHan);
static void RtcProc_Set_UpdateParam(RtcProc_t * pHan,int32_t iMode);
static void RtcProc_SendInfo(RtcProc_t * pHan,sTimeInfo_t * pInfo);
static int32_t RtcProc_SaveInfo(const RtcProc_t * pHan,sTimeInfo_t * pInfo);
static void RtcProc_ProcessKeyEvent(RtcProc_t * pHan,uint8_t iKeyVal);
static int32_t RtcProc_ValidateInfo(const RtcProc_t * pHan,sTimeInfo_t * pInfo);
static void RtcProc_UpdateParam(uint8_t *param, uint8_t minVal, uint8_t maxVal,int32_t iMode);
static void RtcProc_AdjustDayForFebruary(sTimeInfo_t *pInfo);
static void RtcProc_AdjustDayForMonth(sTimeInfo_t *pInfo);
static int32_t RtcProc_ConfigInit(RtcProc_t * pHan);
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_RtcProc_Init(QueueHandle_t RtcQueueId)
{
	int32_t iRval = BCU_OK;

	if (hRtcProc.iIsInitialised != 0 || RtcQueueId == NULL)
	{
		iRval = BCU_NOK;
	}
	else
	{
		memset(&hRtcProc, 0, sizeof(hRtcProc));
		hRtcProc.qHmiMsg = RtcQueueId;
		hRtcProc.qRxMsg = xQueueCreate(MAX_QUEUE_MSG_COUNT, MSG_QUEUE_BUFFER_SIZE);

		iRval = RtcProc_ConfigInit(&hRtcProc);

		if(iRval == BCU_OK)
		{
			Cluster_Controller_Hal_SetReceiveQueueHandle(eModule_Rtc,hRtcProc.qHmiMsg);
			iRval = Cluster_Controller_SwitchProc_SetReceiveQueueHandle(hRtcProc.qRxMsg);
			if (iRval == BCU_OK)
			{
				hRtcProc.iIsInitialised = 1;
				hRtcProc.iState = eStateNone;
			}
		}
	}
	return iRval;
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_RtcProc_DeInit(void)
{
	int32_t iRval = BCU_OK;

	if(hRtcProc.iIsInitialised == 1)
	{
		memset(&hRtcProc, 0, sizeof(RtcProc_t));
	}
	return iRval;
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_RtcProc_GetDateTime(sTimeInfo_t * pInfo)
{
	int32_t iRval = BCU_OK;
	if(pInfo != NULL)
	{
		iRval = Cluster_Controller_Hal_RtcGetDateTime(pInfo);
	}
	return iRval;
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_RtcProc_SetDateTime(uint8_t* pBuf, uint8_t iSize)
{
	int32_t iRval = BCU_OK;
	(void)pBuf;
	(void)iSize;
	return iRval;
}


/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_RtcProc_CheckForValidData(sTimeInfo_t * pRTCInfo )
{
	int32_t iRval = BCU_OK;

	if(pRTCInfo != NULL)
	{
		iRval = RtcProc_ValidateInfo(&hRtcProc,pRTCInfo);
	}
	return iRval;
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
static int32_t RtcProc_ConfigInit(RtcProc_t * pHan)
{
	int32_t iRval = BCU_OK;
	uint16_t iStackSize = 0;
	bool isTaskCreated = false;

	Cluster_Controller_Hal_Config_GetStackSize(&iStackSize);

	if (pHan->qRxMsg != NULL)
	{
		isTaskCreated = (xTaskCreate(RtcProc_ModeHandler,
				"RTProc",
				iStackSize,
				pHan,
				configMAX_PRIORITIES - 2,
				&(pHan->hRTCThread)) == pdPASS);

		if (!isTaskCreated)
		{
			PRINTF("\r\nThe Cluster_Controller_Manager_RtcProc_Init: xTaskCreate failed\r\n");
			iRval = BCU_NOK;
		}
	}
	else
	{
		iRval = BCU_NOK;
	}
	return iRval;
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
static void RtcProc_ModeHandler(void *arg)
{
	RtcProc_t * pHan       = (RtcProc_t *)(arg);
	TickType_t iDelay      = RTC_RECEIVE_DELAY;

	sMessage_t 		sRtcMsg     = {0};
	sNavSwitch_t    sKeyVal 	= {0};

	char sBuf[MSG_QUEUE_BUFFER_SIZE];

	while(1)
	{
		memset(&sBuf[0],0, sizeof(MSG_QUEUE_BUFFER_SIZE));

		memset(&sKeyVal,0, sizeof(sNavSwitch_t));

		if(pdPASS == xQueueReceive(pHan->qRxMsg,&sBuf[0],iDelay))
		{
			memcpy ((char *)&sRtcMsg, &(sBuf[0]), sizeof(sMessage_t));
			if(sRtcMsg.iEventId == eEvent_NavSwitch)
			{
				memcpy((char*)(&sKeyVal),(char *)sRtcMsg.sData,sizeof(sNavSwitch_t));
				RtcProc_ProcessKeyEvent(pHan,sKeyVal.iVal);
			}
		}
		else
		{
			if(pHan->iIsSetMode == 1)
			{
				RtcProc_ValidateInfo(pHan,&sRtcInfo);
				RtcProc_SaveInfo(pHan,&sRtcInfo);
				RtcProc_SendInfo(pHan,&sRtcInfo);
				Cluster_Controller_SwitchProc_RtcSetMode(false);

				iDelay = RTC_RECEIVE_DELAY;
				pHan->iModeCount = 0;
				pHan->iIsSetMode = 0;
				pHan->iState = eStateNone;
			}
		}
	}
}

/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
static void RtcProc_Set_Initialise(RtcProc_t * pHan)
{
	Cluster_Controller_Hal_RtcSetMode(true);

	RtcProc_GetCurrentInfo(pHan);
	pHan->iIsSetMode = 1;
	pHan->iState = eStateDay;
	sRtcInfo.iIsEdit  = 1;

	sRtcInfo.iState = (uint8_t)(pHan->iState);
	RtcProc_SendInfo(pHan,&sRtcInfo);
}

/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
static void RtcProc_Set_SaveChanges(RtcProc_t * pHan)
{
	sRtcInfo.iIsEdit  = 0;
	sRtcInfo.iState   = 0xFF;
	RtcProc_ValidateInfo(pHan,&sRtcInfo);
	RtcProc_SendInfo(pHan,&sRtcInfo);
	RtcProc_SaveInfo(pHan,&sRtcInfo);
	Cluster_Controller_Hal_RtcSetMode(false);

	pHan->iIsSetMode = 0;
	pHan->iState = eStateNone;
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
static void RtcProc_Set_UpdateState(RtcProc_t * pHan)
{
	pHan->iState = pHan->iState + 1;

	if(pHan->iState > eStateMeridiem)
	{
		pHan->iState = eStateDay;
		RtcProc_SaveInfo(pHan,&sRtcInfo);
	}

	if(pHan->iState == eStateHour)
	{
		RtcProc_SaveInfo(pHan,&sRtcInfo);
	}

	sRtcInfo.iIsEdit  = 1;
	sRtcInfo.iState = (uint8_t)(pHan->iState);
	RtcProc_SendInfo(pHan,&sRtcInfo);
}

/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
static void RtcProc_Set_UpdateParam(RtcProc_t *pHan, int32_t iMode)
{
    sRtcInfo.iIsEdit = 1;
    sRtcInfo.iState = pHan->iState;

    switch (pHan->iState)
    {
        case eStateDay:
        	RtcProc_UpdateParam(&sRtcInfo.iDay, 1, 31,iMode);
            break;

        case eStateMonth:
        	RtcProc_UpdateParam(&sRtcInfo.iMonth, 1, 12,iMode);
            break;

        case eStateYear:
        	RtcProc_UpdateParam(&sRtcInfo.iYear, 25, 45,iMode);
            break;

        case eStateHour:
        	RtcProc_UpdateParam(&sRtcInfo.iHour, 1, 12,iMode);
            break;

        case eStateMinute:
        	RtcProc_UpdateParam(&sRtcInfo.iMinute, 0, 59,iMode);
            break;

        case eStateMeridiem:
            hRtcProc.iIsAMorPM++;
            sRtcInfo.bPM = (hRtcProc.iIsAMorPM % 2 == 0);
            break;

        default:
            PRINTF("\r\nError: Invalid Case for RTC update %d\r\n", pHan->iState);
            sRtcInfo.iIsEdit = 0;
            break;
    }

    RtcProc_SendInfo(pHan, &sRtcInfo);
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
static void RtcProc_UpdateParam(uint8_t *param, uint8_t minVal, uint8_t maxVal, int32_t iMode)
{
    if (iMode)
    {
        (*param)++;
        if (*param > maxVal)
        {
            *param = minVal;
        }
    }
    else
    {
        if (*param <= minVal)
        {
            *param = maxVal;
        }
        else
        {
            (*param)--;
        }
    }
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
static void RtcProc_ProcessKeyEvent(RtcProc_t * pHan,uint8_t iKeyVal)
{
	switch(iKeyVal)
	{
		case eRtc_Init:
		{
			RtcProc_Set_Initialise(pHan);
		}
		break;
		case eRtc_State:
		{
			RtcProc_Set_UpdateState(pHan);
		}
		break;
		case eRtc_Param:
		{
			RtcProc_Set_UpdateParam(pHan,1);
		}
		break;
		case eRtc_Save:
		{
			RtcProc_Set_SaveChanges(pHan);
		}
		break;
		default:
		break;
	}
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
static void RtcProc_GetCurrentInfo(const RtcProc_t * pHan)
{
	Cluster_Controller_Hal_RtcGetDateTime(&sRtcInfo);

	PRINTF("The RTC Time [%d: %d: %d :: %d : %d]",sRtcInfo.iDay, sRtcInfo.iMonth, sRtcInfo.iYear, sRtcInfo.iHour,sRtcInfo.iMinute);

	RtcProc_ValidateInfo(pHan,&sRtcInfo);
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
static int32_t RtcProc_ValidateInfo(const RtcProc_t *pHan, sTimeInfo_t *pInfo)
{
	int32_t iRval = BCU_OK;

	if (pHan->iIsInitialised == 1 || pInfo != NULL)
	{
		if (pInfo->iMinute > 59)
		{
			pInfo->iMinute = 59;
		}

		if (pInfo->iHour > 23)
		{
			pInfo->iHour = 23;
		}

		if (pInfo->iYear < 25 || pInfo->iYear == 0 || pInfo->iYear > 45)
		{
			pInfo->iYear = 25;
		}

		if (pInfo->iDay == 0)
		{
			pInfo->iDay = 1;
		}

		if (pInfo->iMonth > 12)
		{
			pInfo->iMonth = 12;
		}
		else if (pInfo->iMonth == 0)
		{
			pInfo->iMonth = 1;
		}

		if (pInfo->iMonth == 2)
		{
			RtcProc_AdjustDayForFebruary(pInfo);
		}
		else
		{
			RtcProc_AdjustDayForMonth(pInfo);
		}
	}
	else
	{
		iRval = BCU_NOK;
	}
	return iRval;
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
static void RtcProc_AdjustDayForFebruary(sTimeInfo_t *pInfo)
{
    int isLeapYear = (pInfo->iYear % 4 == 0) || (pInfo->iYear % 400 == 0);
    uint8_t maxDay = isLeapYear ? 29 : 28;

    if (pInfo->iDay > maxDay)
    {
        pInfo->iDay = maxDay;
    }
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
static void RtcProc_AdjustDayForMonth(sTimeInfo_t *pInfo)
{
	uint8_t maxDay = (pInfo->iMonth == 1 || pInfo->iMonth == 3 || pInfo->iMonth == 5 ||
                  pInfo->iMonth == 7 || pInfo->iMonth == 8 || pInfo->iMonth == 10 ||
                  pInfo->iMonth == 12) ? 31 : 30;

    if (pInfo->iDay > maxDay)
    {
        pInfo->iDay = maxDay;
    }
}

/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
static void RtcProc_SendInfo(RtcProc_t * pHan,sTimeInfo_t* pInfo)
{
	sMessage_t sMsg   = {0};

	if((pHan != NULL) && (pHan->qHmiMsg  != NULL) && (pInfo != NULL))
	{
		sMsg.iEventId     = eEvent_Time;

		memcpy((char *)sMsg.sData,(char*)(pInfo),sizeof(sTimeInfo_t));

		xQueueSend(pHan->qHmiMsg, &sMsg, 100);
	}
}

/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
static int32_t RtcProc_SaveInfo(const RtcProc_t * pHan,sTimeInfo_t * pInfo)
{
	int32_t iRval = BCU_OK;

	if((pHan != NULL)  && (pInfo != NULL))
	{
		RtcProc_ValidateInfo(pHan,pInfo);

		sRtcInfo.iIsEdit  = 1;
		sRtcInfo.iState = (uint8_t)(pHan->iState);

		if(pInfo->bPM == true)
		{
			if(pInfo->iHour == 12)
			{
				pInfo->iHour = 12;
			}
			else
			{
				pInfo->iHour = pInfo->iHour + 12;
			}
		}
		else //if(pInfo->bPM == false)
		{
			if(pInfo->iHour == 12)
			{
				pInfo->iHour = 0;
			}
		}
		PRINTF("The RTC Time [%d: %d: %d :: %d : %d]",pInfo->iDay, pInfo->iMonth, pInfo->iYear, pInfo->iHour,pInfo->iMinute);

		Cluster_Controller_Hal_RtcSetDateTime(pInfo);

		if(pInfo->bPM == true)
		{
			pInfo->iHour = pInfo->iHour - 12;
		}

		if(pInfo->iHour == 0)
		{
			pInfo->iHour = 12;
		}
	}
	return iRval;
}
