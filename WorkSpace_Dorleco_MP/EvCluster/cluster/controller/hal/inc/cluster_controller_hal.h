#ifndef CLUSTER_CONTROLLER_HAL_H_
#define CLUSTER_CONTROLLER_HAL_H_

#ifdef __cplusplus
 extern "C" {
#endif

/*-----------------------------------------------------------------------------*
 *	OptM Media Solutions Confidential										   *
 *	Copyright (C) OptM Media Solutions Pvt. Ltd - All Rights Reserved	   	   *
 *																			   *
 *  Dissemination of this information or reproduction or redistribution of 	   *
 *  this material is  strictly forbidden unless prior written permission is	   *
 *  obtained from OptM Media Solutions Pvt Ltd								   *
 *	File Name	:	cluster_controller_hal.h 								  	   		   
 *  version		: 											              	   *
 *  Date		:	03-Mar-2023													   
 *  Description :  				 										 	   * 
 *                               						                       *
 *																			   *
 *-----------------------------------------------------------------------------*/
 
 
/*----------------------------------------------------------------------------*
 * Include Headers				           	                             	  *
 *----------------------------------------------------------------------------*/
#include <stdio.h>
#include "cluster_common.h"
#include "FreeRTOS.h"
#include "queue.h"


/*----------------------------------------------------------------------------*
 * Structure and enum definations				                              * 
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*
 * Methods                      				                              *
 *----------------------------------------------------------------------------*/
 
 
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Init(int32_t iModules);

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_DeInit(int32_t iModules);
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/ 
int32_t Cluster_Controller_Hal_SetReceiveQueueHandle(int32_t iModule,QueueHandle_t qRxMsg);
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_EnableModule(int32_t iModule,int32_t iEnable);
/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_CanSendMessage  			     	   *
 * Params	    :	Can Message to be sent     		                           *
 * Return value	:	BCU_OK or BCU_NOK				                           *
 * Description	:	sends Message to the device usng CAN                       *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_CanSendMessage(sCanMessage_t *pMsg,uint8_t mbIdx);
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_BTSendMessage(int8_t *pMsg,int32_t iMsgLen);
 
/*----------------------------------------------------------------------------*
  * Function	    :										        	   	      *
  * Params	    :				        		                          	  *
  * Return value	:									                          *
  * Description	:									                          *
  *----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_RtcGetDateTime(sTimeInfo_t * pInfo);

 /*----------------------------------------------------------------------------*
  * Function	    :										        	   	      *
  * Params	    :				        		                          	  *
  * Return value	:									                          *
  * Description	:									                          *
  *----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_RtcSetDateTime(sTimeInfo_t * pInfo);
 
 
 /*-----------------------------------------------------------------------------*
  * Function	    :											        		*
  * Params	    :					        		                            *
  * Return value	:									                        *
  * Description	:							                                    *
  *-----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_SetBrightness(uint8_t iValue);
  /*-----------------------------------------------------------------------------*
    * Function	    :											        		*
    * Params	    :					        		                            *
    * Return value	:									                        *
    * Description	:							                                    *
    *-----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_FlashWriteData(uint8_t * pBuf, int iSize,uint32_t iAddr);
 /*-----------------------------------------------------------------------------*
    * Function	    :											        		*
    * Params	    :					        		                            *
    * Return value	:									                        *
    * Description	:							                                    *
    *-----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_FlashReadData(uint8_t * pBuf, int iSize,uint32_t iAddr);
 /*-----------------------------------------------------------------------------*
    * Function	    :											        		*
    * Params	    :					        		                            *
    * Return value	:									                        *
    * Description	:							                                    *
    *-----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_FlashErase(uint32_t iAddr);
 /*-----------------------------------------------------------------------------*
    * Function	    :											        		*
    * Params	    :					        		                            *
    * Return value	:									                        *
    * Description	:							                                    *
    *-----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_FlashReadProp(int iParam,uint8_t * pBuf, int iSize,uint32_t iAddr);
 /*-----------------------------------------------------------------------------*
    * Function	    :											        		*
    * Params	    :					        		                            *
    * Return value	:									                        *
    * Description	:							                                    *
    *-----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_RtcUpdateDateTime(void);
 /*-----------------------------------------------------------------------------*
  * Function	    :											        		   *
  * Params	    :					        		                           *
  * Return value	:									                           *
  * Description	:							                                   *
  *-----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_SetBrightnessLevel(uint32_t iLevel);

 /*-----------------------------------------------------------------------------*
  * Function	    :											        		   *
  * Params	    :					        		                           *
  * Return value	:									                           *
  * Description	:							                                   *
  *-----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_CanReset();

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
void Cluster_Controller_Hal_FirmwareDownload_Init();
 /*-----------------------------------------------------------------------------*
  * Function	    :											        		   *
  * Params	    :					        		                           *
  * Return value	:									                           *
  * Description	:							                                   *
  *-----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_CanGetMode(void);
 /*-----------------------------------------------------------------------------*
  * Function	    :											        	    *
  * Params	    :					        		                            *
  * Return value	:									                        *
  * Description	:							                                    *
  *-----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_CanEnable(void);
 /*-----------------------------------------------------------------------------*
  * Function	    :											        	    *
  * Params	    :					        		                            *
  * Return value	:									                        *
  * Description	:							                                    *
  *-----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_RtcSetMode(bool bSetMode);
 /*-----------------------------------------------------------------------------*
  * Function	    :											        	    *
  * Params	    :					        		                            *
  * Return value	:									                        *
  * Description	:							                                    *
  *-----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_ConfigGetVersion(char* pBuf,int iSize);
 /*-----------------------------------------------------------------------------*
  * Function	    :											        	    *
  * Params	    :					        		                            *
  * Return value	:									                        *
  * Description	:							                                    *
  *-----------------------------------------------------------------------------*/
 void Cluster_Controller_Hal_ConfigGetSwHwVersion(uint16_t * iSwVersion,uint16_t * iHwVersion);

 /*-----------------------------------------------------------------------------*
  * Function	    :											        		   *
  * Params	    :					        		                           *
  * Return value	:									                           *
  * Description	:							                                   *
  *-----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_HalDisplay_Enable(bool bOnOff);
#ifdef __cplusplus
}
#endif


#endif /* CLUSTER_CONTROLLER_HAL_H_ */
