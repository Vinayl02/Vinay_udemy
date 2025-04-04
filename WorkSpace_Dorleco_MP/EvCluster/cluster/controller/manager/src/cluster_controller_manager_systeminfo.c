/*-----------------------------------------------------------------------------*
 *	OptM Media Solutions Confidential										   *
 *	Copyright (C) OptM Media Solutions Pvt Ltd - All Rights Reserved.	   	   *
 *  Dissemination of this information or reproduction or redistribution of 	   *
 *  this material is  strictly forbidden unless prior written permission is	   *
 *  obtained from OptM Media Solutions Pvt Ltd								   *
 *	File Name	:	cluster_controller_manager_systeminfo.c 							   *
 *  version		: 											              	   *
 *  Date		:	03-Mar-2023												   *
 *  Description :  				 										 	   *
 *                               						                       *
 *-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*
 * Include Headers				           	                             	   *
 *-----------------------------------------------------------------------------*/
#include "fsl_debug_console.h"
#include "cluster_common.h"
#include "semphr.h"
#include "cluster_controller_manager_systeminfo.h"
#include "cluster_controller_hal.h"
#include "cluster_controller_hal_config.h"
/*----------------------------------------------------------------------------*
 * Macro ,Structure and Enum definition                   				      *
 *----------------------------------------------------------------------------*/
#define MAX_QUEUE_MSG_COUNT				4
#define MSG_BUFFER_SIZE					sizeof(sMessage_t)
#define MSG_QUEUE_BUFFER_SIZE 			MSG_BUFFER_SIZE + 16

typedef struct
{
	uint32_t    	iIsInitialised;
	uint32_t        iSectorAddress;
	uint32_t        iProductInfoAddress;
	uint8_t    	    iDTCStatus;
	uint8_t         iReadFlag;
	uint8_t    	    iCurMemPos;
	SemaphoreHandle_t pSysSema;
	sDeviceInfo_t     sProductInfo;
	ClusterInfo_t     sInfoCache;
	TaskHandle_t      hSysInfoThread;
	QueueHandle_t     qRxMsg;
	ClusterInfo_t  	  sMemCache[2];
}SystemInfo;

/*----------------------------------------------------------------------------*
 * Static and global variable definition	                         		  *
 *----------------------------------------------------------------------------*/
SystemInfo hConfig = {0};

static void SysInfo_ModeHandler(void *arg);
static int32_t SystemInfo_GetDTCInfo(SystemInfo * pHan);
static int32_t SystemInfo_GetVehicleSpecInfo(SystemInfo * pHan);
static int32_t SystemInfo_GetClusterData(SystemInfo * pHan);
static int32_t SystemInfo_UpdateClusterData(ClusterInfo_t* pInfo, int32_t iParam,
		uint32_t iSectorAddress,SystemInfo * pHan);
static void SystemInfo_HandleCorruptSectors(SystemInfo *pHan, int iMemSec0Cur, int iMemSec1Cur);
static int32_t SystemInfo_ReadClusterData(ClusterInfo_t *pCache, uint32_t iSector);
/*-----------------------------------------------------------------------------*
 * Function	    : 	Cluster_Controller_Manager_SystemInfo_Init				*
 * Params	        :	void			        		                        *
 * Return value	:	On success return BCU_OK else return BCU_NOK            *
 * Description  	:	In this function initializes the systeminfo and creates
 * 					xSemaphoreCreateBinary to synchronize the read and write
 * 					operation. And Read the data from secondary flash for re
 * 					-ference						                        *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_SystemInfo_Init(void)
{
	int32_t iRval = BCU_OK;
	uint16_t iStackSize = 0;

	if(hConfig.iIsInitialised == 0)
	{
		memset(&hConfig, 0 ,sizeof(SystemInfo));

		hConfig.pSysSema = xSemaphoreCreateBinary();

		Cluster_Controller_Hal_Config_GetStackSize(&iStackSize);

		hConfig.qRxMsg = xQueueCreate(MAX_QUEUE_MSG_COUNT,MSG_QUEUE_BUFFER_SIZE);

		if((hConfig.pSysSema != NULL) && (hConfig.qRxMsg != NULL))
		{
			if (xTaskCreate(SysInfo_ModeHandler,
					"SysInfo",
					iStackSize,
					&hConfig,
					configMAX_PRIORITIES - 2,
					&hConfig.hSysInfoThread) == pdPASS)
			{
				xSemaphoreGive(hConfig.pSysSema);
				hConfig.iIsInitialised = 1;
				SystemInfo_GetClusterData(&hConfig);
			}
		}
	}
	return iRval;
}
/*-----------------------------------------------------------------------------*
 * Function	    : 	Cluster_Controller_Manager_SystemInfo_DeInit			*
 * Params	        :	void			        		                        *
 * Return value	:	On success return BCU_OK else return BCU_NOK            *
 * Description  	:	In this function Deinitializes the systeminfo and delete
 * 					vSemaphore which is created in initialization.			*
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_SystemInfo_DeInit(void)
{
	int32_t iRval = BCU_OK;

	if (xSemaphoreTake(hConfig.pSysSema, portMAX_DELAY) == pdTRUE)
	{
		vSemaphoreDelete(hConfig.pSysSema);
		memset(&hConfig, 0 ,sizeof(SystemInfo));
	}
	return iRval;
}
/*-----------------------------------------------------------------------------*
 * Function	    : 	Cluster_Controller_UpdateSystemInfo						*
 * Params	        :	ClusterInfo_t* pInfo and int32_t iValue			       	*
 * Return value	:	On success return BCU_OK else return BCU_NOK            *
 * Description  	:	In this function,Write the data to secondary flash(Nvm) *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_UpdateSystemInfo(ClusterInfo_t* pInfo, int32_t iValue)
{
	int iRtVal = BCU_NOK;

	if((pInfo != NULL) && (hConfig.iIsInitialised != 0))
	{
		if(hConfig.iCurMemPos == 0)
		{
			PRINTF("In SysInfo Saved data: Odo : %d Trip : %d, MemSector: %d \r\n", pInfo->iOdoVal, pInfo->iCurTrip, hConfig.iCurMemPos);
			uint32_t iSectorAddress = Cluster_Controller_Hal_Config_GetFlashAddress(CLUSTER_INFO_MEMSEC0);
			iRtVal = SystemInfo_UpdateClusterData(pInfo,iValue,iSectorAddress,&hConfig);
			hConfig.iCurMemPos = 1;
		}
		else
		{
			PRINTF("In SysInfo Saved data: Odo : %d Trip : %d, MemSector: %d \r\n", pInfo->iOdoVal, pInfo->iCurTrip, hConfig.iCurMemPos);
			uint32_t iSectorAddress = Cluster_Controller_Hal_Config_GetFlashAddress(CLUSTER_INFO_MEMSEC1);
			iRtVal = SystemInfo_UpdateClusterData(pInfo,iValue,iSectorAddress,&hConfig);
			hConfig.iCurMemPos = 0;
		}
	}
	return iRtVal;
}
/*-----------------------------------------------------------------------------*
 * Function	    : 	Cluster_Controller_GetSystemInfo						*
 * Params	        :	ClusterInfo_t* pInfo		       						*
 * Return value	:	On success return BCU_OK else return BCU_NOK            *
 * Description  	:	In this function,Read the data to secondary flash(Nvm)  *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_GetSystemInfo(ClusterInfo_t* pInfo)
{
	int iRval = BCU_NOK;

	if(hConfig.iIsInitialised != 0)
	{
		memcpy(pInfo,&(hConfig.sInfoCache),sizeof(ClusterInfo_t));
		iRval = BCU_OK;
	}
	return iRval;
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t SystemInfo_UpdateProductInfo(uint64_t iInfo, uint32_t iSectorAddress)
{
	if (xSemaphoreTake(hConfig.pSysSema, portMAX_DELAY) == pdTRUE)
	{
		int iRval = BCU_NOK;

		uint8_t sBuf[128] = {0};

		Cluster_Controller_Hal_FlashErase(iSectorAddress);

		memcpy(sBuf,(uint8_t*)&iInfo,sizeof(iInfo));

		int iSize = sizeof(iInfo);

		iRval = Cluster_Controller_Hal_FlashWriteData(&sBuf[0],
				iSize ,
				iSectorAddress);

		xSemaphoreGive(hConfig.pSysSema);

		return iRval;
	}
	else
	{
		return BCU_NOK;
	}
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_ResetSystemInfo(ClusterInfo_t* pInfo)
{
	int iRtVal = BCU_NOK;

	if((pInfo != NULL) && (hConfig.iIsInitialised != 0))
	{
		uint32_t iSectorAddress = Cluster_Controller_Hal_Config_GetFlashAddress(CLUSTER_INFO_MEMSEC0);

		iRtVal = SystemInfo_UpdateClusterData(pInfo,eResetInfo,hConfig.iSectorAddress,&hConfig);

		iSectorAddress = Cluster_Controller_Hal_Config_GetFlashAddress(CLUSTER_INFO_MEMSEC1);

		iRtVal = SystemInfo_UpdateClusterData(pInfo,eResetInfo,iSectorAddress,&hConfig);
	}
	return iRtVal;
}
/*-----------------------------------------------------------------------------*
 * Function	   	 	: 	Cluster_Controller_SetVehicleSpecInfo						   *
 * Params	        :	  											  		   *
 * Return value		:	On success return BCU_OK else return BCU_NOK           *
 * Description  	:														   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_SetVehicleSpecInfo(void * pInfo, int32_t iParam)
{
	if (xSemaphoreTake(hConfig.pSysSema, portMAX_DELAY) == pdTRUE)
	{
		int iRval = BCU_NOK;
		uint8_t sBuf[128] = {0};
		bool iIsvalid = true;
		switch(iParam)
		{
		case eProductInfo:
		{
			memcpy(&(hConfig.sProductInfo),(sDeviceInfo_t *)pInfo,sizeof(sDeviceInfo_t));
		}
		break;
		case eDTCInfo:
		{
			uint8_t iDtcInfo = 0;

			memcpy(&iDtcInfo,(uint8_t*)pInfo, sizeof(iDtcInfo));

			hConfig.iDTCStatus = (iDtcInfo | hConfig.iDTCStatus);

			uint32_t iDtcInfoAddress = Cluster_Controller_Hal_Config_GetFlashAddress(VEHICLE_DTC_INFO);

			Cluster_Controller_Hal_FlashErase(iDtcInfoAddress);

			int iSize = sizeof(hConfig.iDTCStatus);

			iRval = Cluster_Controller_Hal_FlashWriteData(&(hConfig.iDTCStatus),
					iSize ,
					iDtcInfoAddress);
			iIsvalid = false;
		}
		break;
		default:
			iIsvalid = false;
			break;
		}

		if(iIsvalid)
		{
			uint32_t iProductInfoAddr = Cluster_Controller_Hal_Config_GetFlashAddress(VEHICLE_SEPC_INFO_MEMSEC);

			Cluster_Controller_Hal_FlashErase(iProductInfoAddr);

			memcpy(sBuf,(uint8_t*)&(hConfig.sProductInfo),sizeof(sDeviceInfo_t));
			int iSize = sizeof(sDeviceInfo_t);
			iRval = Cluster_Controller_Hal_FlashWriteData(&sBuf[0],
					iSize ,
					iProductInfoAddr);
		}
		xSemaphoreGive(hConfig.pSysSema);
		return iRval;
	}
	else
	{
		return BCU_NOK;
	}
}
/*-----------------------------------------------------------------------------*
 * Function	   	 	: 	Cluster_Controller_GetVehicleSpecInfo			     	   *
 * Params	        :	void		       									   *
 * Return value		:	On success return BCU_OK else return BCU_NOK           *
 * Description  	:														   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_GetVehicleSpecInfo(sDeviceInfo_t * pInfo)
{
	int iRval = BCU_NOK;
	if((hConfig.iIsInitialised != 0) && (hConfig.iReadFlag == 1))
	{
		memcpy(pInfo,&(hConfig.sProductInfo),sizeof(sDeviceInfo_t));
		iRval = BCU_OK;
	}
	return iRval;
}
/*-----------------------------------------------------------------------------*
 * Function	   	 	: 	Cluster_Controller_GetDTCInfo							   *
 * Params	        :	void		       									   *
 * Return value		:	On success return BCU_OK else return BCU_NOK           *
 * Description  	:														   *
 *-----------------------------------------------------------------------------*/
int32_t  Cluster_Controller_GetDTCInfo(uint8_t * pDtcInfo)
{
	int iRval = BCU_NOK;
	if((hConfig.iIsInitialised != 0) && (hConfig.iReadFlag == 1))
	{
		*pDtcInfo = hConfig.iDTCStatus;
	}
	return iRval;
}
/*-----------------------------------------------------------------------------*
 * Function	   	 	: 	SysInfo_ModeHandler						        	   *
 * Params	        :	void		       									   *
 * Return value		:	On success return BCU_OK else return BCU_NOK           *
 * Description  	:														   *
 *-----------------------------------------------------------------------------*/
static void SysInfo_ModeHandler(void *arg)
{
	int iRval = BCU_NOK;

	SystemInfo * pHan       = (SystemInfo *)(arg);

	TickType_t iDelay      =  2000; //In Millisecond.

	char sBuf[MSG_QUEUE_BUFFER_SIZE];

	memset(&sBuf[0],0, sizeof(MSG_QUEUE_BUFFER_SIZE));

	while(1)
	{
		xQueueReceive(pHan->qRxMsg,(char*)&sBuf[0],iDelay);

		if(pHan->iReadFlag == 0)
		{
			iRval = SystemInfo_GetVehicleSpecInfo(&hConfig);

			if(iRval == BCU_OK)
			{
				iRval = SystemInfo_GetDTCInfo(&hConfig);
				if(iRval == BCU_OK)
				{
					pHan->iReadFlag = 1;
					iDelay = portMAX_DELAY;
				}
			}
		}

		if(iRval == BCU_NOK)
		{
			iDelay = 1000;
		}
	}
}
/*-----------------------------------------------------------------------------*
 * Function	   	 	: 	Cluster_Controller_GetDTCInfo							   *
 * Params	        :	void		       									   *
 * Return value		:	On success return BCU_OK else return BCU_NOK           *
 * Description  	:														   *
 *-----------------------------------------------------------------------------*/
static int32_t SystemInfo_GetDTCInfo(SystemInfo * pHan)
{
	if(xSemaphoreTake(pHan->pSysSema, portMAX_DELAY) == pdTRUE)
	{
		int iRval = BCU_NOK;
		uint8_t sBuf = 0;

		int iSize = sizeof(sBuf);

		uint32_t iDtcInfoAddress = Cluster_Controller_Hal_Config_GetFlashAddress(VEHICLE_DTC_INFO);

		iRval = Cluster_Controller_Hal_FlashReadData(&sBuf,
				iSize ,
				iDtcInfoAddress);

		if(iRval == BCU_OK)
		{
			if(pHan->iDTCStatus != 0xFF)
			{
				memcpy(&(pHan->iDTCStatus),&sBuf,sizeof(sBuf));
			}
			else
			{
				pHan->iDTCStatus = 0;
			}
		}
		xSemaphoreGive(pHan->pSysSema);
		return iRval;
	}
	else
	{
		return BCU_NOK;
	}
}
/*-----------------------------------------------------------------------------*
 * Function	   	 	: 	SystemInfo_GetVehicleSpecInfo							   *
 * Params	        :	void		       									   *
 * Return value		:	On success return BCU_OK else return BCU_NOK           *
 * Description  	:														   *
 *-----------------------------------------------------------------------------*/
static int32_t SystemInfo_GetVehicleSpecInfo(SystemInfo * pHan)
{
	if(xSemaphoreTake(pHan->pSysSema, portMAX_DELAY) == pdTRUE)
	{
		int iRval = BCU_NOK;
		uint8_t sBuf[128] = {0};

		memset(&(pHan->sProductInfo), 0,sizeof(sDeviceInfo_t));

		int iSize = sizeof(sDeviceInfo_t);

		uint32_t iVehicleInfoAddr = Cluster_Controller_Hal_Config_GetFlashAddress(VEHICLE_SEPC_INFO_MEMSEC);

		iRval = Cluster_Controller_Hal_FlashReadData(&sBuf[0],
				iSize ,
				iVehicleInfoAddr);

		if(iRval == BCU_OK)
		{
			if((sBuf[0] == 0xFF) && (sBuf[1] == 0xFF)
					&& (sBuf[2] == 0xFF) && (sBuf[3] == 0xFF))
			{
				PRINTF("\r\nThe Device Info is not Set\r\n");
			}
			else
			{
				memcpy((char *)(&(pHan->sProductInfo)),(char*)(&sBuf[0]),sizeof(sDeviceInfo_t));
			}
		}
		xSemaphoreGive(pHan->pSysSema);
		return (iRval);
	}
	else
	{
		return BCU_NOK;
	}
}
/*-----------------------------------------------------------------------------*
 * Function	   	 	: 	SystemInfo_GetClusterData							   *
 * Params	        :	void		       									   *
 * Return value		:	On success return BCU_OK else return BCU_NOK           *
 * Description  	:	In this function,Read the data to secondary flash(Nvm) *
 *-----------------------------------------------------------------------------*/
static int32_t SystemInfo_GetClusterData(SystemInfo *pHan)
{
    int iRval = BCU_NOK;
    memset(&(pHan->sInfoCache), 0, sizeof(ClusterInfo_t));

    bool bMemSec0Cur = SystemInfo_ReadClusterData(&pHan->sMemCache[0], CLUSTER_INFO_MEMSEC0);
    bool bMemSec1Cur = SystemInfo_ReadClusterData(&pHan->sMemCache[1], CLUSTER_INFO_MEMSEC1);

    SystemInfo_HandleCorruptSectors(pHan, bMemSec0Cur, bMemSec1Cur);
    ClusterInfo_t  *pSelectedCache = (pHan->sMemCache[0].iOdoVal >= pHan->sMemCache[1].iOdoVal) ?
    		                          &pHan->sMemCache[0] : &pHan->sMemCache[1];
    memcpy(&(pHan->sInfoCache), pSelectedCache, sizeof(ClusterInfo_t));

    return iRval;
}
/*-----------------------------------------------------------------------------*
 * Function	   	 	: 	SystemInfo_UpdateClusterData						   *
 * Params	        :	ClusterInfo_t* pInfo, int32_t iParam		  		   *
 * Return value		:	On success return BCU_OK else return BCU_NOK           *
 * Description  	:	In this function,Write the data to secondary flash(Nvm)*
 *-----------------------------------------------------------------------------*/
static int32_t SystemInfo_UpdateClusterData(ClusterInfo_t* pInfo, int32_t iParam,
		uint32_t iSectorAddress, SystemInfo * pHan)
{
	if (xSemaphoreTake(pHan->pSysSema, portMAX_DELAY) == pdTRUE)
	{
		int iRval = BCU_NOK;
		uint8_t sBuf[128] = {0};
		bool iIsvalid = true;
		switch(iParam)
		{
		case eSpeedInfo:
		{
			pHan->sInfoCache.iOdoVal = pInfo->iOdoVal;
			pHan->sInfoCache.iCurTrip = pInfo->iCurTrip;
		}
		break;
#if 0
		case eBrightnessInfo:
		{
			pHan->sInfoCache.iBrightness = pInfo->iBrightness;
		}
		break;
		case eBLEStatusInfo:
		{
			pHan->sInfoCache.iBleStatus = pInfo->iBleStatus;
		}
		break;
		case eGsmStatusInfo:
		{
			pHan->sInfoCache.iGsmStatus = pInfo->iGsmStatus;
		}
#endif
		break;
		case eResetInfo:
		{
			memset(&(pHan->sInfoCache), 0,sizeof(ClusterInfo_t));
		}
		break;
		default:
			iIsvalid = false;
			break;
		}

		if(iIsvalid)
		{
			Cluster_Controller_Hal_FlashErase(iSectorAddress);
			memcpy(sBuf,(uint8_t*)&(pHan->sInfoCache),sizeof(ClusterInfo_t));
			int iSize = sizeof(ClusterInfo_t);
			iRval = Cluster_Controller_Hal_FlashWriteData(&sBuf[0],
					iSize ,
					iSectorAddress);
		}
		xSemaphoreGive(pHan->pSysSema);
		return (iRval);
	}
	else
	{
		return BCU_NOK;
	}
}
/*-----------------------------------------------------------------------------*
 * Function	   	 	: 	SystemInfo_ReadClusterData   						   *
 * Params	        :	                                        	  		   *
 * Return value		:	On success return BCU_OK else return BCU_NOK           *
 * Description  	:														   *
 *-----------------------------------------------------------------------------*/
static int32_t SystemInfo_ReadClusterData(ClusterInfo_t *pCache, uint32_t iSector)
{
	int iRval = BCU_NOK;
    uint8_t sBuf[sizeof(ClusterInfo_t)] = {0};
    uint32_t iSectorAddress = Cluster_Controller_Hal_Config_GetFlashAddress(iSector);

    iRval = Cluster_Controller_Hal_FlashReadData(sBuf, sizeof(ClusterInfo_t), iSectorAddress);
    if (iRval == BCU_OK)
    {
        memcpy(pCache, sBuf, sizeof(ClusterInfo_t));
        PRINTF("\r\n\033[33mIn SysInfo sector %d Cached Data :OdO:\033[0m\033[32m[ %d] \033[33m, trip:\033[0m\033[32m[ %d]\033[0m\r\n",
               iSector, pCache->iOdoVal, pCache->iCurTrip);
    }
    else
    {
        PRINTF("\r\nError: Failed to read sector %d \r\n", iSector);
    }

    return (pCache->iOdoVal == 0xFFFFFFFF && pCache->iCurTrip == 0xFFFFFFFF) ? 1 : 0;
}
/*-----------------------------------------------------------------------------*
 * Function	   	 	: 	SystemInfo_HandleCorruptSectors 					   *
 * Params	        :	                                        	  		   *
 * Return value		:	                                                       *
 * Description  	:														   *
 *-----------------------------------------------------------------------------*/
static void SystemInfo_HandleCorruptSectors(SystemInfo *pHan, int iMemSec0Cur, int iMemSec1Cur)
{
    if(iMemSec0Cur && iMemSec1Cur)
    {
        PRINTF("\r\nIn SysInfo: The Device is New or NVM is Corrupted\r\n");
        Cluster_Controller_ResetSystemInfo(&pHan->sInfoCache);
    }
    else if(iMemSec0Cur)
    {
        uint32_t iSectorAddress = Cluster_Controller_Hal_Config_GetFlashAddress(CLUSTER_INFO_MEMSEC0);
        SystemInfo_UpdateClusterData(&pHan->sMemCache[1], eSpeedInfo, iSectorAddress, &hConfig);
        PRINTF("\r\nIn SysInfo: NVM Sector0 is Corrupted\r\n");
    }
    else if(iMemSec1Cur)
    {
        uint32_t iSectorAddress = Cluster_Controller_Hal_Config_GetFlashAddress(CLUSTER_INFO_MEMSEC1);
        SystemInfo_UpdateClusterData(&pHan->sMemCache[0], eSpeedInfo, iSectorAddress, &hConfig);
        PRINTF("\r\nIn SysInfo: NVM Sector1 is Corrupted\r\n");
    }
}
