#ifndef CLUSTER_CONTROLLER_MANAGER_CANPROC_H_
#define CLUSTER_CONTROLLER_MANAGER_CANPROC_H_


#ifdef __cplusplus
 extern "C" {
#endif

/*-----------------------------------------------------------------------------*
 *	OptM Media Solutions Confidential										   *
 *	Copyright (C) OptM Media Solutions Pvt. Ltd - All Rights Reserved	   	   *
 *																			   *
 *  Dissemination of this information or reproduction or redistribution of 	   *
 *  this material is  strictly forbidden unless prior written permission is	   *
 *  oCanained from OptM Media Solutions Pvt Ltd								   *
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
 int32_t Cluster_Controller_Manager_CanProc_Init(QueueHandle_t CanQueueId);

 /*----------------------------------------------------------------------------*
  * Function	    :										        	   	   *
  * Params	    :				        		                          	   *
  * Return value	:									                       *
  * Description	:									                           *
  *----------------------------------------------------------------------------*/
 void Cluster_Controller_Manager_CanProc_DeInit(void);
 /*----------------------------------------------------------------------------*
  * Function	    :										        	   	   *
  * Params	    :				        		                          	   *
  * Return value	:									                       *
  * Description	:									                           *
  *----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Manager_CanProc_SendMessage(sCanMessage_t *pMsg,uint8_t mbIdx);
 /*----------------------------------------------------------------------------*
  * Function	    :										        	   	   *
  * Params	    :				        		                          	   *
  * Return value	:									                       *
  * Description	:									                           *
  *----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Manager_CanProc_Start(void);
 /*----------------------------------------------------------------------------*
  * Function	    :										        	   	   *
  * Params	    :				        		                          	   *
  * Return value	:									                       *
  * Description	:									                           *
  *----------------------------------------------------------------------------*/

 int32_t Cluster_Controller_Manager_CanProc_Stop(void);
 /*----------------------------------------------------------------------------*
  * Function	    :										        	   	   *
  * Params	    :				        		                          	   *
  * Return value	:									                       *
  * Description	:									                           *
  *----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Manager_CanProc_Reset(void);
 /*----------------------------------------------------------------------------*
  * Function	    :										        	   	   *
  * Params	    :				        		                          	   *
  * Return value	:									                       *
  * Description	:									                           *
  *----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Manager_CanProc_ResetTrip(void);
 /*----------------------------------------------------------------------------*
  * Function	    :										        	   	   *
  * Params	    :				        		                          	   *
  * Return value	:									                       *
  * Description	:									                           *
  *----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Manager_CanProc_IgnitionStatus(uint8_t iStatus);
 /*----------------------------------------------------------------------------*
  * Function	    :										        	   	   *
  * Params	    :				        		                          	   *
  * Return value	:									                       *
  * Description	:									                           *
  *----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Manager_CanProc_SetUdsQueueHan(QueueHandle_t qRxMsg);
 /*----------------------------------------------------------------------------*
  * Function	    :										        	   	   *
  * Params	    :				        		                          	   *
  * Return value	:									                       *
  * Description	:									                           *
  *----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Manager_CanProc_ProductionReset(void);
 /*----------------------------------------------------------------------------*
  * Function	    :										        	   	   *
  * Params	    :				        		                          	   *
  * Return value	:									                       *
  * Description	:									                           *
  *----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Manager_CanProc_GetCanFrame(uint8_t * pFrame,
		 	 	 	 	 	 	 	 	 	 	 	    uint8_t iFrameId);

#ifdef __cplusplus
}
#endif


#endif /* CLUSTER_CONTROLLER_MANAGER_CANPROC_H_ */
