/*-----------------------------------------------------------------------------*
 *	OptM Media Solutions Confidential										   *
 *	Copyright (C) OptM Media Solutions Pvt Ltd - All Rights Reserved.	   	   *
 *  Dissemination of this information or reproduction or redistribution of 	   *
 *  this material is  strictly forbidden unless prior written permission is	   *
 *  obtained from OptM Media Solutions Pvt Ltd								   *
 *	File Name	:	cluster_controller_udsproc.c 							   *
 *  version		: 											              	   *
 *  Date		:	03-Mar-2023												   *
 *  Description :  				 										 	   *
 *                               						                       *
 *-----------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------*
 * Include Headers				           	                             	   *
 *-----------------------------------------------------------------------------*/
#include "stdlib.h"
#include "time.h"
#include "fsl_flexcan.h"
#include "cluster_common.h"
#include "fsl_debug_console.h"
#include "cluster_controller_manager_udsproc.h"
#include "cluster_controller_manager_canproc.h"
#include "cluster_controller_hal_config.h"
#include "cluster_controller_hal_can.h"
#include "cluster_controller_hal.h"
#include "cluster_controller_manager_switchproc.h"
#include "cluster_controller_manager_rtcproc.h"
#include "cluster_controller_manager_systeminfo.h"
/*----------------------------------------------------------------------------*
* Macro ,Structure and Enum definitions           				              *
*-----------------------------------------------------------------------------*/
#define UDS_MAX_QUEUE_MSG_COUNT				150
#define UDS_MSG_BUFFER_SIZE					sizeof(flexcan_frame_t)

typedef struct
{
	uint32_t    	 iIsInitialised;
	uint32_t         iSeedValue;
	uint32_t         iKeyValue;
	uint8_t          iWriteSeqCheck;
	bool             iIsSecurityEnable;
	uint8_t          sInfoCacheData[32];
	QueueHandle_t    qRxMsg;
	TaskHandle_t 	 hRxThread;
	flexcan_frame_t* pFrame;
	sDeviceInfo_t    sSysInfo;
	sDeviceInfo_t    sSysInfoCache;
}UdsProc_t;
/*----------------------------------------------------------------------------*
 * Static and global variable definition	                         		  *
 *----------------------------------------------------------------------------*/
UdsProc_t hUdsProc = {0};
/*----------------------------------------------------------------------------*
 * Local function definition	                         				      *
 *----------------------------------------------------------------------------*/
static void UDS_DelayMs(uint32_t ms);
static void UdsProc_DataHandler(void *arg);
static int32_t UdsProc_GenNegativeResp(int32_t ilen, int32_t iSid, int32_t iNrc);
static void UdsProc_ParseServiceRequest(UdsProc_t * pHan,flexcan_frame_t* pFrame);
static int32_t UdsProc_GenPositiveResp(int32_t ilen, int32_t iSid, int32_t iSubFunid, uint32_t iData);
static int  UDSProc_SendCommand(uint8_t *pdata,uint8_t mbIdx);
static int  UDSProc_ComposeCommnd(uint8_t * pCmd, void * vpData);
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_UdsProc_Init()
{
	int32_t iRval = BCU_NOK;
	BaseType_t iTaskRetval = 0;

	if(hUdsProc.iIsInitialised == 0)
	{
		uint16_t iStackSize = 0;

		hUdsProc.qRxMsg = xQueueCreate( UDS_MAX_QUEUE_MSG_COUNT,
										UDS_MSG_BUFFER_SIZE);
		if(hUdsProc.qRxMsg != 0)
		{
			Cluster_Controller_Manager_CanProc_SetUdsQueueHan(hUdsProc.qRxMsg);

			Cluster_Controller_Hal_Config_GetStackSize(&iStackSize);

			iTaskRetval = xTaskCreate(UdsProc_DataHandler,
					"UDSP",
					iStackSize*2,
					&hUdsProc,
					(configMAX_PRIORITIES - 2),
					&hUdsProc.hRxThread);
			if (iTaskRetval == pdPASS)
			{
				hUdsProc.iIsInitialised = 1;
			}
		}
		else
		{
			PRINTF("\r\n ERROR: xQueueCreate\r\n");
		}
	}
	return iRval;
}
/*----------------------------------------------------------------------------*
 * Function	    :													 		  *
 * Params	    :						       							      *
 *			    :   			         		                              *
 * Return value	:														  	  *
 * Description	:								                              *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_UdsProc_DeInit(void)
{
	int32_t iRval = BCU_NOK;

	if(hUdsProc.iIsInitialised == 1)
	{
		hUdsProc.iIsInitialised = 0;
	}

	return iRval;
}
/*----------------------------------------------------------------------------*
 * Function	    :													 		  *
 * Params	    :						       							      *
 *			    :   			         		                              *
 * Return value	:														  	  *
 * Description	:								                              *
 *----------------------------------------------------------------------------*/
static void UdsProc_DataHandler(void *arg)
{
	UdsProc_t *  pHan    = (UdsProc_t *)(arg);
	flexcan_frame_t sMsg = {0};

	while(1)
	{
		memset(&sMsg, 0, sizeof(flexcan_frame_t));

		if(pdPASS == xQueueReceive(pHan->qRxMsg,(char*)(&sMsg),(TickType_t)(1000)))
		{
			UdsProc_ParseServiceRequest(pHan,&(sMsg));
		}
		else
		{
			UDS_DelayMs(1000);
		}
	}
}
/*----------------------------------------------------------------------------*
 * Function	    :													 		  *
 * Params	    :						       							      *
 *			    :   			         		                              *
 * Return value	:														  	  *
 * Description	:								                              *
 *----------------------------------------------------------------------------*/
static void UdsProc_ParseServiceRequest(UdsProc_t * pHan,flexcan_frame_t* pFrame)
{
	uint8_t iLen       = pFrame->dataByte0;
	uint8_t iServiceID = pFrame->dataByte1;
	uint8_t iSubFunID  = pFrame->dataByte2;

	switch(iServiceID)
	{
		case eSecurityAccess:
		{
			switch(iSubFunID)
			{
				case eRequestSeed:
				{
					if(iLen == 0x02)
					{
						srand((unsigned int)time(NULL));

						pHan->iSeedValue = (uint32_t)(rand() & 0xFFFF);

						pHan->iKeyValue  = ((~pHan->iSeedValue + 1) & 0xFFFF);

						UdsProc_GenPositiveResp(4, iServiceID,iSubFunID,pHan->iSeedValue);
					}
					else
					{
						UdsProc_GenNegativeResp(iLen,iServiceID, eIncorrectMesLenOrInavlidFormat);
					}
				}
				break;
				case eSendKey:
				{
					if(iLen == 0x04)
					{
						uint32_t iIsValidKey = (uint32_t)(pFrame->dataByte4 << 8 | pFrame->dataByte3);

						if(pHan->iKeyValue == iIsValidKey)
						{
							pHan->iIsSecurityEnable = true;

							UdsProc_GenPositiveResp(2,iServiceID,iSubFunID,0);
						}
						else
						{
							UdsProc_GenNegativeResp(2,iServiceID, eInvalidKey);
						}
					}
					else
					{
						UdsProc_GenNegativeResp(iLen,iServiceID, eIncorrectMesLenOrInavlidFormat);
					}
				}
				break;
				default:
					UdsProc_GenNegativeResp(3,iServiceID, eSubFunNotSupported);
					break;
			}
		}
		break;
		case eDiagnosticSessionControl:
		{
			if(iSubFunID == eProgrammingSession)
			{
				UdsProc_GenPositiveResp(1,iServiceID,0,0);
				Cluster_Controller_Hal_FirmwareDownload_Init();
			}
			else
			{
				UdsProc_GenNegativeResp(3,iServiceID, eConditiosNotCorrect);
			}
		}
		break;
		case eRtcSetTime:
		{
			sTimeInfo_t sRTCInfo = {0};

			sRTCInfo.iSecond = pFrame->dataByte1;
			sRTCInfo.iMinute = pFrame->dataByte2;
			sRTCInfo.iHour = pFrame->dataByte3;
			sRTCInfo.iDay = pFrame->dataByte4;
			sRTCInfo.iMonth = pFrame->dataByte5;
			sRTCInfo.iYear = pFrame->dataByte6;

			Cluster_Controller_Manager_RtcProc_CheckForValidData(&sRTCInfo);

			PRINTF("\r\n The RTC frame: [%x %x %x %x %x %x]",sRTCInfo.iSecond,sRTCInfo.iMinute,sRTCInfo.iHour,sRTCInfo.iDay,sRTCInfo.iMonth,sRTCInfo.iYear);

			Cluster_Controller_Hal_RtcSetDateTime(&sRTCInfo);

			UdsProc_GenPositiveResp(1,iServiceID,0,0);
		}
		break;
		case eHardwareStatus:
		{
			int32_t iRval = BCU_NOK;
			uint8_t sData[8] = {0};
			sHwInputs_t sHwStatus = {0};
			sCanMessage_t CanMsg;

			iRval = Cluster_Controller_SwitchProc_GetHwStatus(&sHwStatus);

			if (iRval == BCU_OK)
			{
				sData[0] = 0x03;
				sData[1] = (uint8_t)iServiceID + 0x40;

				switch(iSubFunID)
				{
				case eInput1:
					sData[2] = sHwStatus.bInput2;
					break;

				case eInput2:
					sData[2] = sHwStatus.bInput3;
					break;

				case eInput3:
					sData[2] = sHwStatus.bInput4;
					break;

				case eInput4:
					sData[2] = sHwStatus.bInput5;
					break;

				case eInput5:
					sData[2] = sHwStatus.bInput6;
					break;

				case eInput6:
					sData[2] = sHwStatus.bInput7;
					break;

				case eInput7:
					sData[2] = sHwStatus.bInput8;
					break;

				case eInput8:
					sData[2] = sHwStatus.bInput9;
					break;

				case eInput9:
					sData[2] = sHwStatus.bInput10;
					break;

				case eInput10:
					sData[2] = sHwStatus.bInput11;
					break;

				case eInputs_All:
					sData[2] = (uint8_t)(((sHwStatus.bInput8 & 0x01) << 7) | ((sHwStatus.bInput7 & 0x01) << 6)
							| ((sHwStatus.bInput6 & 0x01) << 5) | ((sHwStatus.bInput5 & 0x01) << 4)
							| ((sHwStatus.bInput4 & 0x01) << 3) | ((sHwStatus.bInput3 & 0x01) << 2)
							| ((sHwStatus.bInput2 & 0x01)));
					sData[3] = (uint8_t)(((sHwStatus.bInput10 & 0x01) << 1) | (sHwStatus.bInput9 & 0x01) | (sHwStatus.bInput11 & 0x01));
					break;

				default:
					PRINTF("\r\nError: Unknown Sub-function ID\r\n");
					return;
				}
				CanMsg.iCanId   = eCANID_UDSTx;
				CanMsg.iFormat  = 0;
				CanMsg.iDataLen = 8;

				memcpy(CanMsg.sData,sData,sizeof(CanMsg.sData));

				iRval = Cluster_Controller_Hal_CanSendMessage(&CanMsg,15);

				if(iRval != BCU_OK)
				{
					PRINTF("\r\nError: CAN_PROC eHardwareStatus CAN Send failed\r\n");
				}
			}
		}
		break;
		case eFactoryReset:
		{
			int32_t iRtval = BCU_NOK;
			iRtval = Cluster_Controller_Manager_CanProc_ProductionReset();
			if(iRtval == BCU_OK)
			{
				PRINTF("\r\nThe Production Reset successful\r\n");
				UdsProc_GenPositiveResp(1,iServiceID,0,0);
			}
		}
		break;
		case eReadSwHwVersion:
		{
			int32_t iRval = BCU_NOK;
			uint8_t sData[8] = {0};
			uint16_t iSwVersion = 0;
			uint16_t iHwVersion = 0;
			sCanMessage_t CanMsg  = {0};

			Cluster_Controller_Hal_ConfigGetSwHwVersion(&iSwVersion, &iHwVersion);

			sData[0] = (uint8_t)0x03;
			sData[1] = (uint8_t)iServiceID + 0x40;
			sData[2] = (uint8_t)iSwVersion;
			sData[3] = (uint8_t)iHwVersion;

			CanMsg.iCanId   = eCANID_UDSTx;
			CanMsg.iFormat  = 0;
			CanMsg.iDataLen = 8;

			memcpy(CanMsg.sData,sData,sizeof(CanMsg.sData));

			iRval = Cluster_Controller_Hal_CanSendMessage(&CanMsg,15);

			if(iRval != BCU_OK)
			{
				PRINTF("\r\nError: CAN_PROC eHardwareStatus CAN Send failed\r\n");
			}
		}
		break;
		case eWriteDataByIdentifier:
		{
			if(pFrame->dataByte3 == 0x01)
			{
				if(pHan->iWriteSeqCheck == 0)
				{
					pHan->iWriteSeqCheck += 1;

					Cluster_Controller_GetVehicleSpecInfo(&(pHan->sSysInfoCache));

					pHan->sInfoCacheData[0] = pFrame->dataByte4;
					pHan->sInfoCacheData[1] = pFrame->dataByte5;
					pHan->sInfoCacheData[2] = pFrame->dataByte6;
					pHan->sInfoCacheData[3] = pFrame->dataByte7;

					UdsProc_GenPositiveResp(3,eWriteDataByIdentifier,0x01,0);
				}
				else //sequence error
				{
					pHan->iWriteSeqCheck = 0;
					UdsProc_GenNegativeResp(2,eWriteDataByIdentifier,eConditiosNotCorrect);
				}
			}
			else if(pFrame->dataByte3 == 0x02)
			{
				if(pHan->iWriteSeqCheck == 1)
				{
					pHan->iWriteSeqCheck += 1;

					pHan->sInfoCacheData[4] = pFrame->dataByte4;
					pHan->sInfoCacheData[5] = pFrame->dataByte5;
					pHan->sInfoCacheData[6] = pFrame->dataByte6;
					pHan->sInfoCacheData[7] = pFrame->dataByte7;

					UdsProc_GenPositiveResp(3,eWriteDataByIdentifier,0x02,0);
				}
				else
				{
					pHan->iWriteSeqCheck = 0;
					UdsProc_GenNegativeResp(2,eWriteDataByIdentifier,eConditiosNotCorrect);
				}
			}
			else if(pFrame->dataByte3 == 0x03)
			{
				if(pHan->iWriteSeqCheck == 2)
				{
					uint8_t iIdentifier = iSubFunID;

					pHan->sInfoCacheData[8] = pFrame->dataByte4;
					pHan->sInfoCacheData[9] = pFrame->dataByte5;
					pHan->sInfoCacheData[10] = pFrame->dataByte6;
					pHan->sInfoCacheData[11] = pFrame->dataByte7;

					memcpy(&(pHan->sSysInfo),&(pHan->sSysInfoCache),sizeof(pHan->sSysInfo));

				if(iIdentifier == eSlNoIdentifier) //SlNum.
						{
					pHan->sSysInfo.iSlNum = ((uint32_t)pHan->sInfoCacheData[3] << 24) |
							((uint32_t)pHan->sInfoCacheData[2] << 16) |
							((uint32_t)pHan->sInfoCacheData[1] << 8)  |
							(uint32_t)pHan->sInfoCacheData[0];

					pHan->sSysInfoCache.iSlNum = pHan->sSysInfo.iSlNum;

					/*memcpy(pHan->sSysInfo.iSlNum,pHan->sInfoCacheData, sizeof(pHan->sSysInfo.iSlNum));

						memcpy(pHan->sSysInfoCache.iSlNum,pHan->sSysInfo.iSlNum, sizeof(pHan->sSysInfo.iSlNum));*/
						}
				else if(iIdentifier == eVINIdentifier) //VIN.
				{
					memcpy(pHan->sSysInfo.sVin, pHan->sInfoCacheData, sizeof(pHan->sSysInfo.sVin));

						memcpy(pHan->sSysInfoCache.sVin,pHan->sSysInfo.sVin, sizeof(pHan->sSysInfo.sVin));
					}


					Cluster_Controller_SetVehicleSpecInfo(&(pHan->sSysInfo), eProductInfo);

					memset(&(pHan->sSysInfo), 0 , sizeof(pHan->sSysInfo));
					memset(pHan->sInfoCacheData, 0 , sizeof(pHan->sInfoCacheData));

					UdsProc_GenPositiveResp(3,eWriteDataByIdentifier,0x03,0);
					pHan->iWriteSeqCheck = 0;
				}
				else
				{
					pHan->iWriteSeqCheck = 0;
					UdsProc_GenNegativeResp(2,eWriteDataByIdentifier,eConditiosNotCorrect);
				}
			}
		}
		break;
		case eReadDataByIdentifier:
		{
			uint8_t sData[8] = {0};

			sDeviceInfo_t sInfo = {0};

			uint8_t iIdentifier = iSubFunID;

			Cluster_Controller_GetVehicleSpecInfo(&sInfo);

		if((sInfo.iSlNum == 0xFFFFFFFF) && ((unsigned char) sInfo.sVin[0] == 0xFF))
		{
			PRINTF("\r\nThe Device is New Set the Device details\r\n");
		}
		else
		{
			if(iIdentifier == eSlNoIdentifier)
			{
				sData[0] = (uint8_t)sizeof(sInfo.iSlNum);
				sData[1] = iServiceID + 0x40;
				sData[2] = eSlNoIdentifier;

				UDSProc_ComposeCommnd(sData, &sInfo.iSlNum);
			}
			else if(iIdentifier == eVINIdentifier)
			{
				sData[0] =  (uint8_t)strlen(sInfo.sVin);
				sData[1] = iServiceID + 0x40;
				sData[2] = eVINIdentifier;

				UDSProc_ComposeCommnd(sData, sInfo.sVin);
			}
		}
	}
	break;
	case eReadDTCInformation:
	{
		if(iSubFunID == eReportDTCByStatusMask)
		{
			//To do, Check for the DTC status availability mask

				uint8_t sData[8] = {0};
				uint8_t iDtcFaults = 0;

				Cluster_Controller_GetDTCInfo(&iDtcFaults);

				sData[0] = (uint8_t)0x06;
				sData[1] = (uint8_t)iServiceID + 0x40;
				sData[2] = (uint8_t)0x00; //DTC 3 byte higher byte
				sData[3] = (uint8_t)0x00; //DTC 3 byte middle byte

				for(int i = 0; i <= 4; i++)
				{
					 switch(i)
					 {
					 	 case 0:
					 	 {
					 		sData[4] = (uint8_t)eCanFault; //DTC 3 byte lower byte
					 		if((iDtcFaults && eCanFault) == eCanFault)
					 		{
					 			sData[5] = (uint8_t)0x04;
					 		}
					 	 }
					 	 break;
					 	 case 1:
					 	 {
					 		 sData[4] = (uint8_t)eRtcFault;
					 		 if((iDtcFaults & eRtcFault) == eRtcFault)
					 		 {
					 			 sData[5] = (uint8_t)0x04;
					 		 }
					 	 }
					 	 break;
					 	 case 2:
					 	 {
					 		 sData[4] = (uint8_t)eSwitchFault;
					 		 if((iDtcFaults & eSwitchFault) == eSwitchFault)
					 		 {
					 			 sData[5] = (uint8_t)0x04;
					 		 }
					 	 }
					 	 break;
					 	 case 3:
					 	 {
					 		sData[4] = (uint8_t)eBleFault;
					 		if((iDtcFaults & eBleFault) == eBleFault)
					 		{
					 			sData[5] = (uint8_t)0x04;
					 		}
					 	 }
					 	 break;
					 	 case 4:
					 	 {
					 		 sData[4] = (uint8_t)eGsmFault;
					 		 if((iDtcFaults & eGsmFault) == eGsmFault)
					 		 {
					 			 sData[5] = (uint8_t)0x04;
					 		 }
					 	 }
					 	 break;
					 	 default:
					 		 break;
					 }
					 UDSProc_SendCommand(sData,15);
					 sData[5] = (uint8_t)0x00;
				}
			}
			else
			{
				UdsProc_GenNegativeResp(2,eReadDTCInformation,eSubFunNotSupported);
			}
		}
		break;
		case eClearDiagnosticInformation:
		{
			uint8_t iClearDTC = 0;
			//To do, Check for the field to clear DTC.
		    Cluster_Controller_SetVehicleSpecInfo(&iClearDTC,eDTCInfo);
			UdsProc_GenPositiveResp(1,eClearDiagnosticInformation,0,0);
		}
		break;
		default:
			PRINTF("\r\n DataParse Default  SID: 0x%x\r\n",iServiceID);
			break;
	}
}
/*----------------------------------------------------------------------------*
 * Function	    :													 		  *
 * Params	    :						       							      *
 *			    :   			         		                              *
 * Return value	:														  	  *
 * Description	:								                              *
 *----------------------------------------------------------------------------*/
static int32_t UdsProc_GenNegativeResp(int32_t ilen, int32_t iSid, int32_t iNrc)
{
	int iRetVal = BCU_OK;
	uint8_t  sData[8] = {0};

	sData[0] = (uint8_t)ilen;
	sData[1] = (uint8_t)0x7F;
	sData[2] = (uint8_t)iSid;
	sData[3] = (uint8_t)iNrc;

	UDSProc_SendCommand(sData,15);

	return iRetVal;
}
/*----------------------------------------------------------------------------*
 * Function	    :													 		  *
 * Params	    :						       							      *
 *			    :   			         		                              *
 * Return value	:														  	  *
 * Description	:								                              *
 *----------------------------------------------------------------------------*/
static int32_t UdsProc_GenPositiveResp(int32_t ilen, int32_t iSid, int32_t iSubFunid, uint32_t iData)
{
	int iRetVal = BCU_OK;
	uint8_t  sData[8] = {0};

	sData[0] = (uint8_t)ilen;
	sData[1] = (uint8_t)iSid + 0x40;
	sData[2] = (uint8_t)iSubFunid;

	if(iData != 0)
	{
		sData[3] = (uint8_t)iData & 0xFF;
		sData[4] = (uint8_t)(iData >> 8) & 0xFF;
	}

	UDSProc_SendCommand(sData,15);

	return iRetVal;
}
/*----------------------------------------------------------------------------*
  Function	      	:	UDSProc_SendCommand
  Params	    	:
  Return value		:
  Description		:
 *----------------------------------------------------------------------------*/
static int  UDSProc_SendCommand(uint8_t *pdata,uint8_t mbIdx)
{
	int iRetVal = BCU_OK;

	sCanMessage_t CanMsg;

    CanMsg.iCanId   = eCANID_UDSTx;
	CanMsg.iFormat  = 0;
	CanMsg.iDataLen = 8;

	memcpy(CanMsg.sData,pdata,sizeof(CanMsg.sData));

	iRetVal = Cluster_Controller_Hal_CanSendMessage(&CanMsg,mbIdx);

	for (uint8_t m = 0; m < 8; m++)
	{
		PRINTF("%02X ", CanMsg.sData[m]); //For reference
	}
	PRINTF("\n");

	return iRetVal;
}
/*----------------------------------------------------------------------------*
  Function	      	:	UDSProc_ComposeCommnd
  Params	    	:
  Return value		:
  Description		:
 *----------------------------------------------------------------------------*/
static int  UDSProc_ComposeCommnd(uint8_t * pCmd, void * vpData)
{
	int32_t iRval = BCU_NOK;
	uint8_t i = 4;
	uint8_t k = 0;
	if(pCmd[0] != eSlNoIdentifier)
	{
		char * pData = vpData;
		for(uint8_t j = 0; j < 12; j++)
		{
			pCmd[i] = pData[j];
			i += 1;
			if(i == 8)
			{
				i = 4;
				k += 1;
				pCmd[i-1] = k;

			for (uint8_t m = 0; m < 8; m++)
			{
				PRINTF("%02X ", pCmd[m]); //For reference
			}

				iRval = UDSProc_SendCommand(&pCmd[0],15);

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

		iRval = UDSProc_SendCommand(&pCmd[0],15);

		if(iRval != BCU_OK)
		{
			PRINTF("\r\nError: CAN_PROC READ_SWHW_VERSION CAN Send failed\r\n");
		}
	}

	return iRval;
}
/*----------------------------------------------------------------------------*
 * Function	      	:	UDS_DelayMs				          	  	              *
 * Params	    	:								       		              *
 * Return value		:								                          *
 * Description		:							                              *
 *----------------------------------------------------------------------------*/
static void UDS_DelayMs(uint32_t ms)
{
#if defined(SDK_OS_FREE_RTOS)
	TickType_t tick;

	tick = ms * configTICK_RATE_HZ / 1000U;

	tick = (0U == tick) ? 1U : tick;

	vTaskDelay(tick);
#else
	while (0U != (ms--))
	{
		SDK_DelayAtLeastUs(1000U, SystemCoreClock);
	}
#endif
}
