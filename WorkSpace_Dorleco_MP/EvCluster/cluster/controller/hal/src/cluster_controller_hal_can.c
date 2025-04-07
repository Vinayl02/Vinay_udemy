/*-----------------------------------------------------------------------------*
 *	OptM Media Solutions Confidential										   *
 *	Copyright (C) OptM Media Solutions Pvt Ltd - All Rights Reserved.	   	   *
 *  Dissemination of this information or reproduction or redistribution of 	   *
 *  this material is  strictly forbidden unless prior written permission is	   *
 *  obtained from OptM Media Solutions Pvt Ltd								   *
 *	File Name	:	cluster_controller_hal_can.c 							   *
 *  version		: 											              	   *
 *  Date		:	03-Mar-2023												   *
 *  Description :  				 										 	   * 
 *                               						                       *
 *-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*
 * Include Headers				           	                             	   *
 *-----------------------------------------------------------------------------*/

#include "cluster_common.h"
#include "MIMXRT1176_cm7.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "clock_config.h"
#include "fsl_flexcan.h"
#include "cluster_controller_hal_can.h"
#include "cluster_controller_hal_config.h"

/*----------------------------------------------------------------------------*
 * Macro ,Structure and enum definition                   				      *
 *----------------------------------------------------------------------------*/
#define CAN_MESSAGE_BUFFER_NUM (10U)

typedef struct
{
	int32_t 			  iIsInitialised;
	sCanConfig_t * 		  pCanIdList;
	uint32_t			  iMaxCanId;
	uint8_t				  iTxmbIdx;
    IRQn_Type			  IRqn;
	QueueHandle_t         qRxMsg;
	sCanMsg_t			  sMsg[CAN_MESSAGE_BUFFER_NUM];
	sCanMsg_t			  sErrMsg;
    flexcan_handle_t 	  hFlexCan;
	flexcan_mb_transfer_t rxXfer;
    CAN_Type *            pCanType;
	volatile bool 		  bWakenUp;
	int32_t               iIsDataReceived;
}hal_can_t;

/*----------------------------------------------------------------------------* 
 * Static and global variable definition	                         		  * 
 *----------------------------------------------------------------------------*/
static hal_can_t   hCan = {0};

/*----------------------------------------------------------------------------* 
 * Local function definition	                         				      * 
 *----------------------------------------------------------------------------*/
static int  CAN_Configure(hal_can_t * pHan);
static int  CAN_CallBackHandler_Init(hal_can_t * pHan);
static void Can_MessageBuffer_Enable(hal_can_t * pHan, bool bEnable);
void CAN_ReceiveCallBack(CAN_Type *base, flexcan_handle_t *handle,
					     status_t status, uint64_t result,
						 void *userData);

/*-----------------------------------------------------------------------------*
 * Function	    :				Cluster_Controller_Hal_Can_Init       		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Can_Init(void)
{
    int32_t iRval = BCU_OK;
    if(hCan.iIsInitialised == 0)
    {
		memset(&hCan, 0 ,sizeof(hal_can_t));
		iRval = CAN_Configure(&hCan);
		if(iRval == BCU_OK )
		{
			hCan.iIsInitialised = 1;
			hCan.bWakenUp = false;
		}
		else
		{
			PRINTF("CAN Configure Failed:\r\n");
		}
    }
    return (iRval);
}
/*-----------------------------------------------------------------------------*
 * Function	    :			Cluster_Controller_Hal_Can_DeInit        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Can_DeInit(void)
{
    int32_t iRval = BCU_OK;
    if(hCan.iIsInitialised == 1)
    {
    	memset(&hCan, 0, sizeof(hal_can_t));
    }
    return (iRval);
}
/*-----------------------------------------------------------------------------*
 * Function	    :			Cluster_Controller_Hal_Can_SetReceiveQueueHandle   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Can_SetReceiveQueueHandle(QueueHandle_t  hCanQueue)
{
	int32_t iRval = BCU_NOK;
	if ((hCan.iIsInitialised > 0) && (hCanQueue != NULL))
	{
		hCan.qRxMsg	= hCanQueue;
	}
	else
	{
		PRINTF("Error: Invalid Queue handle:\r\n");
	}
	return (iRval);
}
/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Can_Enable		        		   *
 * Params	    :					        		                           *
 * Return value	:	BCU_OK or BCU_NOK				                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Can_Enable()
{
	int32_t iRval 		= BCU_NOK;
	if(hCan.qRxMsg != NULL)
	{
		iRval= CAN_CallBackHandler_Init(&hCan);
		if(iRval != BCU_OK)
		{
			PRINTF("ERROR:CAN_CallBackHandler_Init FAILED:\r\n");
		}
	}
	return (iRval);
}

/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Can_Enable		        		   *
 * Params	    :					        		                           *
 * Return value	:	BCU_OK or BCU_NOK				                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Can_Reset()
{
	int32_t iRval 		= BCU_OK;

	NVIC_DisableIRQ(hCan.IRqn);

	Can_MessageBuffer_Enable(&hCan,false);

	NVIC_ClearPendingIRQ(hCan.IRqn);

	Can_MessageBuffer_Enable(&hCan,true);

	NVIC_EnableIRQ(hCan.IRqn);

	return (iRval);
}
/*-----------------------------------------------------------------------------*
 * Function	    :			Cluster_Controller_Hal_Can_SendMessage     		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Can_SendMessage(sCanMessage_t *pMsg, uint8_t mbIdx)
{
	int32_t iRval = BCU_NOK;
	status_t iStatus;
	flexcan_frame_t txFrame = {0};

	if((hCan.iIsInitialised > 0) && (pMsg != NULL) && (hCan.bWakenUp == true))
    {
		FLEXCAN_SetTxMbConfig(hCan.pCanType, mbIdx, true);
		// TBD  recheck TX_MESSAGE_BUFFER_NUM
		hCan.iTxmbIdx = mbIdx;

		//txFrame.format = (uint8_t)kFLEXCAN_FrameFormatStandard;
		txFrame.format = (pMsg->iFormat != 0) ? kFLEXCAN_FrameFormatExtend : kFLEXCAN_FrameFormatStandard; //(uint32_t)(pMsg->iFormat);
		txFrame.type   = (uint32_t)kFLEXCAN_FrameTypeData;

		if(pMsg->iFormat == 0)
		{
			txFrame.id     = FLEXCAN_ID_STD(pMsg->iCanId);
		}
		else
		{
			txFrame.id     = FLEXCAN_ID_EXT(pMsg->iCanId);
		}
		txFrame.length = (pMsg->iDataLen & 0x0F); //(uint32_t)pMsg->iDataLen;

		// Case Idatalen is < 8, To Be handled

		txFrame.dataWord0 = CAN_WORD0_DATA_BYTE_0(pMsg->sData[0]) | 
							CAN_WORD0_DATA_BYTE_1(pMsg->sData[1]) |
							CAN_WORD0_DATA_BYTE_2(pMsg->sData[2]) |
							CAN_WORD0_DATA_BYTE_3(pMsg->sData[3]);

		txFrame.dataWord1 = CAN_WORD1_DATA_BYTE_4(pMsg->sData[4]) | 
							CAN_WORD1_DATA_BYTE_5(pMsg->sData[5]) |
							CAN_WORD1_DATA_BYTE_6(pMsg->sData[6]) |
							CAN_WORD1_DATA_BYTE_7(pMsg->sData[7]);

		iStatus = FLEXCAN_TransferSendBlocking(hCan.pCanType,mbIdx, &txFrame);
		if(iStatus == kStatus_Success)
		{
			iRval = BCU_OK;
		}
		else
		{
			PRINTF("Error : CAN Send Failed : Status =  %d \r\n",iStatus);
		}
	}
	return iRval;
}
/*-----------------------------------------------------------------------------*
 * Function	    :				CAN_Configure				        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
static int CAN_Configure(hal_can_t * pHan)
{
	int32_t iRetVal = BCU_OK;
	int32_t iId =0;
	uint32_t iBaud =0;
	uint32_t uCanClkFreq = 0;
	flexcan_config_t sConfig = {0};
	clock_root_config_t rootCfg = {0};

	iRetVal = Cluster_Controller_Hal_Config_CanGetInterface(&iId,&iBaud);
	if(iRetVal == BCU_OK)
	{
		rootCfg.mux   = 1;   // OSC24Mhz as master flexcan clock source
		rootCfg.div   = 1;   // Clock divider for master flex can clock source
		switch(iId)
		{
			case 1:
			{
				CLOCK_SetRootClock(kCLOCK_Root_Can1, &rootCfg);
				pHan->pCanType = CAN1;
				pHan->IRqn = CAN1_IRQn;
				uCanClkFreq =  ((CLOCK_GetRootClockFreq(kCLOCK_Root_Can1) / 100000U) * 100000U);
			}
			break;
			case 2:
			{
				CLOCK_SetRootClock(kCLOCK_Root_Can2, &rootCfg);
				pHan->pCanType = CAN2;
				pHan->IRqn = CAN2_IRQn;
				uCanClkFreq =  ((CLOCK_GetRootClockFreq(kCLOCK_Root_Can2) / 100000U) * 100000U);
			}
			break;
			case 3:
			{
				CLOCK_SetRootClock(kCLOCK_Root_Can3, &rootCfg);
				pHan->pCanType = CAN3;
				pHan->IRqn = CAN3_IRQn;
				uCanClkFreq =  ((CLOCK_GetRootClockFreq(kCLOCK_Root_Can3) / 100000U) * 100000U);
			}
			break;
			default:
			{
				CLOCK_SetRootClock(kCLOCK_Root_Can1, &rootCfg);
				pHan->pCanType = CAN1;
				pHan->IRqn = CAN1_IRQn;
				uCanClkFreq =  ((CLOCK_GetRootClockFreq(kCLOCK_Root_Can1) / 100000U) * 100000U);
			}
			break;
		}	

		FLEXCAN_GetDefaultConfig(&sConfig);

		//flexcanConfig.enableIndividMask = true;  //Default is false
		//flexcanConfig.clkSrc = kFLEXCAN_ClkSrc0;   //Default
		sConfig.maxMbNum = 32;
		sConfig.bitRate = iBaud;

		flexcan_timing_config_t sTimeConfig = {0};
		memset(&sTimeConfig, 0, sizeof(flexcan_timing_config_t));

		iRetVal = FLEXCAN_CalculateImprovedTimingValues(pHan->pCanType,
														sConfig.bitRate,
														 uCanClkFreq,
														 &sTimeConfig);
		if (iRetVal)
		{
			memcpy(&(sConfig.timingConfig), &sTimeConfig, sizeof(flexcan_timing_config_t));
			iRetVal = BCU_OK;
		}

		FLEXCAN_Init(pHan->pCanType, &sConfig, uCanClkFreq);
	}
	return iRetVal;
}

/*-----------------------------------------------------------------------------*
 * Function	    :				CAN_ReceiveCallBack			        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
void CAN_ReceiveCallBack(CAN_Type *base, flexcan_handle_t *handle,
					     status_t status, uint64_t result,
						 void *userData)
{
	(void)base;
	(void)handle;
	hal_can_t * pHanCb = (hal_can_t * )(userData);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint8_t iId =0;
	switch (status)
	{
	case kStatus_FLEXCAN_RxIdle:
			{
				pHanCb->bWakenUp = true;
				pHanCb->iIsDataReceived = 1;
				if ((result > 0) &&  (result <= pHanCb->iMaxCanId))
				{
					iId =(uint8_t)(result-1);
					pHanCb->sMsg[iId].iEventId =0;
					if(pHanCb->qRxMsg != NULL)
					{
						if(pdFALSE == xQueueIsQueueFullFromISR(pHanCb->qRxMsg))
						{

							bool rval = xQueueSendFromISR(pHanCb->qRxMsg,
									(void*)(&(pHanCb->sMsg[iId])),
									&xHigherPriorityTaskWoken);


							if( rval != pdPASS )
							{
								NVIC_ClearPendingIRQ(pHanCb->IRqn);
							}
						}
						else
						{
							PRINTF("\r\n The Queue is FULL\r\n");
						}
					}
					pHanCb->rxXfer.mbIdx = (uint8_t)(result);
					pHanCb->rxXfer.frame = &(pHanCb->sMsg[iId].sRxFrame);
					FLEXCAN_TransferReceiveNonBlocking((pHanCb->pCanType),
															    &(pHanCb->hFlexCan),
																&(pHanCb->rxXfer));
				}
			}
			break;

			case kStatus_FLEXCAN_TxIdle:
			{
				if ((pHanCb->iTxmbIdx == result))
				{
					//pHanCb->txComplete = true;
				}
				pHanCb->bWakenUp = true;
			}
			break;

			case kStatus_FLEXCAN_WakeUp:
				pHanCb->bWakenUp = true;
			break;

			default:
			{
				if(status == kStatus_FLEXCAN_ErrorStatus)
				{
					pHanCb->bWakenUp = false;
				}

				if((pHanCb->qRxMsg != NULL) && (pHanCb->iIsDataReceived == 1))
				{
					pHanCb->sErrMsg.iEventId = 0xFF;
					pHanCb->sErrMsg.iErrId = status;

					if(pdFALSE == xQueueIsQueueFullFromISR(pHanCb->qRxMsg))
					{
						xQueueSendFromISR(pHanCb->qRxMsg,
								(void*)(&(pHanCb->sErrMsg)),
								&xHigherPriorityTaskWoken);
					}
					else
					{
						PRINTF("\r\n The Queue is FULL\r\n");
					}
				}
			}
			break;
		}

	}
/*-----------------------------------------------------------------------------*
 * Function	    :				Can_SetDefaultFilters		        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
#if 0
static int32_t Can_SetDefaultFilters(hal_can_t * pHan)
{
//	PRINTF("Can_SetDefaultFilters Enters\r\n");

	int32_t 	iRval 				= BCU_NOK;
	int32_t 	i 					= 0;
	int32_t * 	pCanIdList			= NULL;
	pHan->sFilter = NULL;

	pHan->iMaxCanId = Cluster_Controller_Hal_Config_GetMaxCanId();

	//int32_t 	iFiltersSize		= pHan->iMaxCanId * sizeof(CAN_filter);


//	CAN_filter sFilter[pHan->iMaxCanId];
	pCanIdList = Cluster_Controller_Hal_Config_GetCanIdList();
	//pHan->pCanIdList = Cluster_Controller_Hal_Config_GetCanIdList();
	if((pCanIdList != NULL) && (pHan->sFilter != NULL))
	{
		for(i =0; i < pHan->iMaxCanId ; i++)
		{
			pHan->sFilter[i].can_id   = pCanIdList[i] & CAN_SFF_MASK;
			pHan->sFilter[i].can_mask = CAN_SFF_MASK;
		}

		iRval = BCU_OK;
	}
	return iRval;
}
#endif
/*-----------------------------------------------------------------------------*
 * Function	    :				Can_SetRX_MessageBuffer		        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
static void Can_MessageBuffer_Enable(hal_can_t * pHan, bool bEnable)
{
	flexcan_rx_mb_config_t mbConfig ={0};

    memset(&(pHan->sMsg),	0, 	sizeof(sCanMsg_t));
	memset(&(pHan->rxXfer),	0, 	sizeof(flexcan_mb_transfer_t));

	memset(&mbConfig,	0, 	sizeof(flexcan_rx_mb_config_t));

    mbConfig.type = kFLEXCAN_FrameTypeData;

    pHan->pCanIdList = Cluster_Controller_Hal_Config_GetCanIdList();
    pHan->iMaxCanId = Cluster_Controller_Hal_Config_GetMaxCanId();

    for(uint32_t i=0; i < (pHan->iMaxCanId);i++)
	{
		mbConfig.format = pHan->pCanIdList[i].iMode;
		if(mbConfig.format == kFLEXCAN_FrameFormatExtend)
		{
			mbConfig.id = FLEXCAN_ID_EXT(pHan->pCanIdList[i].iId);
		}
		else
		{
			mbConfig.id = FLEXCAN_ID_STD(pHan->pCanIdList[i].iId);
		}
		FLEXCAN_SetRxMbConfig(pHan->pCanType, (uint8_t)(i+1), &mbConfig, bEnable);
	}

    FLEXCAN_SetTxMbConfig(pHan->pCanType, hCan.iTxmbIdx, bEnable);

    if(bEnable)
    {
		for(uint32_t i=1; i<=(pHan->iMaxCanId); i++)
		{
			pHan->rxXfer.mbIdx = (uint8_t)(i);
			pHan->rxXfer.frame = &(pHan->sMsg[i-1].sRxFrame);
			(void)FLEXCAN_TransferReceiveNonBlocking(pHan->pCanType,
													 &(pHan->hFlexCan),
													 &(pHan->rxXfer));
		}
    }
}
/*-----------------------------------------------------------------------------*
 * Function	    :				CanProc_CleanUp				        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
static int CAN_CallBackHandler_Init(hal_can_t * pHan)
{
	int32_t 	iRetVal = BCU_OK;

	FLEXCAN_TransferCreateHandle(pHan->pCanType,
								  &(pHan->hFlexCan),
								  CAN_ReceiveCallBack,
								  &hCan);

	Can_MessageBuffer_Enable(pHan,true);

	NVIC_SetPriority(pHan->IRqn, 3);
	NVIC_ClearPendingIRQ(pHan->IRqn);
	NVIC_EnableIRQ(pHan->IRqn);

	return iRetVal;
}
