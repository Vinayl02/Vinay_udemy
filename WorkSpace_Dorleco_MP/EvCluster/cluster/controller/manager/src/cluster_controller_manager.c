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
#include "string.h"
#include "fsl_debug_console.h"
#include "cluster_common.h"
#include "cluster_controller_manager.h"
#include "cluster_controller_hal.h"
#include "cluster_controller_manager_canproc.h"
#include "cluster_controller_manager_switchproc.h"
#include "cluster_controller_manager_systeminfo.h"
#include "cluster_controller_manager_rtcproc.h"
#include "cluster_controller_manager_udsproc.h"
/*----------------------------------------------------------------------------* 
 * Macro ,Structure and Enum definition                   				      *
 *----------------------------------------------------------------------------*/
typedef struct
{
	int32_t 			iIsInitialised;
	int32_t				iModuleStatus;
	int32_t				iStartStatus;
	QueueHandle_t	    iEventTxmq;
} Manager_t;

/*----------------------------------------------------------------------------* 
 * Static and global variable definition	                         		  * 
 *----------------------------------------------------------------------------*/
#if (UDS_SERVICE_ENABLE == 0)
#define DEFAULT_ENABLE_MODULES eModule_Can|eModule_Switch|eModule_Flash|eModule_Rtc
#else
#define DEFAULT_ENABLE_MODULES eModule_Can|eModule_Switch|eModule_Flash|eModule_Rtc|eModule_Uds
#endif

#if ENABLE_PWM_CONTROL
	#define INIT_MODULES  	DEFAULT_ENABLE_MODULES |eModule_BackLight
#else
	#define INIT_MODULES  	DEFAULT_ENABLE_MODULES
#endif

static Manager_t   hMgr = {0};

/*----------------------------------------------------------------------------* 
 * Local function definition	                         				      * 
 *----------------------------------------------------------------------------*/
static void   	 Manager_Proc_Init(Manager_t*   pHan,int32_t iModule);
static void   	 Manager_Proc_DeInit(Manager_t* phan,int32_t iModules);
static int32_t   Manager_Modules_Stop(Manager_t* phan,int32_t iModule);
static int32_t   Manager_Modules_Start(Manager_t* phan);
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_Init(sController_t * pCtl)
{
	int32_t iRval = BCU_OK;
#if ENABLE_PWM_CONTROL
	ClusterInfo_t sInfo = {0};
#endif
	if(hMgr.iIsInitialised == 0)
	{
		memset(&hMgr,0, sizeof(Manager_t));
		hMgr.iEventTxmq = (QueueHandle_t)pCtl->QueueId;
		hMgr.iModuleStatus = Cluster_Controller_Hal_Init(INIT_MODULES);
		if(hMgr.iModuleStatus != 0)
		{
			char version[64] = {0};
			Cluster_Controller_Hal_ConfigGetVersion(&version[0],64);
			PRINTF("\r\n\n %s\r\n\n",version);

			Manager_Proc_Init(&hMgr,INIT_MODULES);
#if ENABLE_PWM_CONTROL
			iRval = Cluster_Controller_GetSystemInfo(&sInfo);
			if(iRval == BCU_OK )
			{
				Cluster_Controller_Manager_Display_Set_Brightness(sInfo.iBrightness);
			}
#endif

			hMgr.iIsInitialised = 1;
		}
		else
		{
			iRval = BCU_NOK;
		}
	}
	return (iRval);
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_DeInit(void)
{
	if(hMgr.iIsInitialised == 1)
	{	
		memset(&hMgr, 0, sizeof(Manager_t));
		Manager_Proc_DeInit(&hMgr,INIT_MODULES);
		Cluster_Controller_Hal_DeInit(INIT_MODULES);
	}
	return (0);
}

/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_Start()
{
	int32_t iRval = BCU_OK;		
	iRval = Manager_Modules_Start(&hMgr);
	return (iRval);
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_Stop(int32_t iModules)
{
	int32_t iRval = BCU_OK;	
	iRval = Manager_Modules_Stop(&hMgr,iModules);
	return(iRval);
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_Set_QueueHandle(int32_t iModules,QueueHandle_t iMsgQ)
{
	int32_t iRval = BCU_NOK;
	(void)iModules;
	(void)iMsgQ;
	return (iRval);
}

/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_RtcGetDateTime(sTimeInfo_t * pInfo)
{
	int32_t iRval = BCU_OK;
	iRval = Cluster_Controller_Manager_RtcProc_GetDateTime(pInfo);
	return (iRval);
}

/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_RtcSetDateTime(uint8_t* pBuf, uint8_t iSize)
{
	int32_t iRval = BCU_OK;
	(void)pBuf;
	(void)iSize;
	return (iRval);
}
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_Display_Set_BrightnessLevel(uint32_t iLevel)
{
	int32_t iRval = BCU_NOK;
	if(iLevel == 0)
	{
		iLevel = 1; // set min value to 1 to avoid complete blank
	}
	iRval = Cluster_Controller_Hal_SetBrightnessLevel(iLevel);
	return (iRval);
}

/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_Display_Set_Brightness(uint8_t iValue)
{
	int32_t iRval = BCU_NOK;
	if(iValue == 0)
	{
		iValue = 1; // set min value to 1 to avoid complete blank
	}
	else if(iValue > 90)
	{
		iValue = 90;
	}
	iRval = Cluster_Controller_Hal_SetBrightness(iValue);
	return (iRval);
}
/*----------------------------------------------------------------------------*
 * Local function 				                         				      *
 *----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*
 * Function	    :										        			  *
 * Params	    :						        		                      *
 * Return value	:	Void							                          *
 * Description	:							                                  *
 *----------------------------------------------------------------------------*/
static int32_t Manager_Modules_Start(Manager_t* pMgr)
{
	(void)pMgr;
	int32_t iRval = BCU_OK;
	iRval = Cluster_Controller_Manager_CanProc_Start();
	return (iRval);
}
/*----------------------------------------------------------------------------*
 * Function	    :										        			  *
 * Params	    :						        		                      *
 * Return value	:	Void							                          *
 * Description	:							                                  *
 *----------------------------------------------------------------------------*/
static int32_t Manager_Modules_Stop(Manager_t* pMgr,int32_t iModule)
{
	(void)pMgr;
	(void)iModule;
	return(0);
}

/*----------------------------------------------------------------------------*
 * Function	    :										        			  *
 * Params	    :						        		                      *
 * Return value	:	Void							                          *
 * Description	:							                                  *
 *----------------------------------------------------------------------------*/
static void Manager_Proc_Init(Manager_t* phan,int32_t iModules)
{
	(void)iModules;

	Cluster_Controller_Manager_SystemInfo_Init();
	
	if(phan->iModuleStatus & eModule_Can )
	{
		Cluster_Controller_Manager_CanProc_Init(phan->iEventTxmq);
	}

	if(phan->iModuleStatus & eModule_Switch )
	{
		Cluster_Controller_Manager_SwitchProc_Init(phan->iEventTxmq);
	}
	if(phan->iModuleStatus & eModule_Rtc )
	{
		Cluster_Controller_Manager_RtcProc_Init(phan->iEventTxmq);
	}
	if(phan->iModuleStatus & eModule_Uds )
	{
		Cluster_Controller_Manager_UdsProc_Init();
	}
}
/*----------------------------------------------------------------------------*
 * Function	    :										        			  *
 * Params	    :						        		                      *
 * Return value	:	Void							                          *
 * Description	:							                                  *
 *----------------------------------------------------------------------------*/
static void Manager_Proc_DeInit(Manager_t* phan,int32_t iModules)
{
	(void)phan;
	(void)iModules;

	Cluster_Controller_Manager_CanProc_DeInit();
	Cluster_Controller_Manager_SwitchProc_DeInit();
    Cluster_Controller_Manager_RtcProc_DeInit();

    if(phan->iModuleStatus & eModule_Uds )
    {
    	Cluster_Controller_Manager_UdsProc_DeInit();
    }
}
