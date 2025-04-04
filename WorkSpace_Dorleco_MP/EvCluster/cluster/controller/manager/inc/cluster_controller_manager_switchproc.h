#ifndef CLUSTER_CONTROLLER_MANAGER_SWITCHPROC_H_
#define CLUSTER_CONTROLLER_MANAGER_SWITCHPROC_H_


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
 *	File Name	:	cluster_hal_spi.h
 *  version		: 											              	   *
 *  Date		:	03-Mar-2023
 *  Description :  				 										 	   *
 *                               						                       *
 *																			   *
 *-----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*
 * Include Headers				           	                             	  *
 *----------------------------------------------------------------------------*/



/*----------------------------------------------------------------------------*
 * Structure and enum definations				                              *
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*
 * Methods                      				                              *
 *----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Manager_SwitchProc_Init(QueueHandle_t QueueId);
 /*----------------------------------------------------------------------------*
  * Function	    :										        	   	      *
  * Params	    :				        		                          	  *
  * Return value	:									                          *
  * Description	:									                          *
  *----------------------------------------------------------------------------*/
 void Cluster_Controller_Manager_SwitchProc_DeInit(void);
 /*----------------------------------------------------------------------------*
  * Function	    :										        	   	   *
  * Params	    :				        		                          	   *
  * Return value	:									                       *
  * Description	:									                           *
  *----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_SwitchProc_SetReceiveQueueHandle(QueueHandle_t qRxMsg);
 /*----------------------------------------------------------------------------*
  * Function	    :										        	   	   *
  * Params	    :				        		                          	   *
  * Return value	:									                       *
  * Description	:									                           *
  *----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_SwitchProc_GetHwStatus(sHwInputs_t* pStatus);
 /*----------------------------------------------------------------------------*
  * Function	    :										        	   	   *
  * Params	    :				        		                          	   *
  * Return value	:									                       *
  * Description	:									                           *
  *----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_SwitchProc_RtcSetMode(bool bSetMode);
#ifdef __cplusplus
}
#endif


#endif /* CLUSTER_CONTROLLER_MANAGER_BTPROC_H_ */
