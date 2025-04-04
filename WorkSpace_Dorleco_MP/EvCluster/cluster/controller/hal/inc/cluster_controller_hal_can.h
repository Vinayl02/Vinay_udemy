#ifndef CLUSTER_CONTROLLER_HAL_CAN_H_
#define CLUSTER_CONTROLLER_HAL_CAN_H_


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
 *	File Name	:	cluster_controller_hal_can.h 								  	   		   
 *  version		: 											              	   *
 *  Date		:	03-Mar-2023													   
 *  Description :  				 										 	   * 
 *                               						                       *
 *																			   *
 *-----------------------------------------------------------------------------*/
 /*----------------------------------------------------------------------------*
 * Include Headers				           	                             	  *
 *----------------------------------------------------------------------------*/
#include "cluster_common.h"
#include "fsl_flexcan.h"
/*----------------------------------------------------------------------------*
 * Structure and enum definations				                              * 
 *----------------------------------------------------------------------------*/

 typedef struct
 {
     int32_t iEventId;
     int32_t iErrId;
     flexcan_frame_t   sRxFrame;
 }sCanMsg_t;

/*----------------------------------------------------------------------------*
 * Methods                      				                              *
 *----------------------------------------------------------------------------*/
 
/*-----------------------------------------------------------------------------*
 * Function	    :	Bcu_Cluster_Hal_Can_Init					        		   *
 * Params	    :					        		                           *
 * Return value	:	BCU_OK or BCU_NOK				                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Can_Init(void);
/*-----------------------------------------------------------------------------*
 * Function	    :	Bcu_Cluster_Hal_Bt_DeInit				        		   *
 * Params	    :					        		                           *
 * Return value	:	BCU_OK or BCU_NOK				                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Can_DeInit(void);
 
/*-----------------------------------------------------------------------------*
 * Function	    :	Bcu_Cluster_Hal_Bt_SetReceiveQueueHandle        		   *
 * Params	    :					        		                           *
 * Return value	:	BCU_OK or BCU_NOK				                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Can_SetReceiveQueueHandle(QueueHandle_t BtQueueID);
/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Can_Enable		        		   *
 * Params	    :					        		                           *
 * Return value	:	BCU_OK or BCU_NOK				                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Can_Enable();
/*-----------------------------------------------------------------------------*
 * Function	    :	Bcu_Cluster_Hal_Bt_ReceiveMessage			        	   *
 * Params	    :					        		                           *
 * Return value	:	BCU_OK or BCU_NOK				                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Can_SendMessage(sCanMessage_t *pCanMessage,uint8_t mbIdx);
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Can_Reset();
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Can_GetMode(void);

#ifdef __cplusplus
}
#endif


#endif /* CLUSTER_CONTROLLER_HAL_CAN_H_ */
