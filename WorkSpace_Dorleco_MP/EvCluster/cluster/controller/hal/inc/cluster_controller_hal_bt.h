#ifndef CLUSTER_CONTROLLER_HAL_BT_H_
#define CLUSTER_CONTROLLER_HAL_BT_H_


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
 *	File Name	:	cluster_controller_hal_uart.h 								  	   		   
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

/*----------------------------------------------------------------------------*
 * Structure and enum definations				                              * 
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*
 * Methods                      				                              *
 *----------------------------------------------------------------------------*/
 
 /*-----------------------------------------------------------------------------*
  * Function	    :	Cluster_Controller_Hal_Bt_Init						    *
  * Params	   	    :	void				        		                    *
  * Return value	:	On success return BCU_OK else return BCU_NOK			*
  * Description	    :	In this method, Initialization and configuration of uart
  * 					will take place for blue tooth low energy communication.*
  *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Bt_Init(void);
/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Bt_DeInit		        		   *
 * Params	    :	void				        		                       *
 * Return value	:	On success return BCU_OK else return BCU_NOK			   *
 * Description	:	In this method, Deinitialization of uart and cancellation
 *					will take place for blue tooth low energy communication.   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Bt_DeInit(void);
 
/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Bt_SetReceiveQueueHandle			   *
 * Params	    :	QueueHandle_t  hBtQueue				        		       *
 * Return value	:	On success return BCU_OK else return BCU_NOK			   *
 * Description	:	In this method, Creating of thread to get the data from
 * 					uart, and accessing of queue id to send data to ble proc.  *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Bt_SetReceiveQueueHandle(QueueHandle_t BtQueueID);
/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Bt_Enable		        		   *
 * Params	    :	int32_t iEnable				        		               *
 * Return value	:	On success return BCU_OK else return BCU_NOK		       *
 * Description	:	In this method, Enable and disable of BLE module will take
 * 					place.						                               *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Bt_Enable(int32_t iEnable);
/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Bt_SendMessage					   *
 * Params	    :	int8_t *pMsg, int32_t iLen				        		   *
 * Return value	:	On success return BCU_OK else return BCU_NOK			   *
 * Description	:	In this method, The sending of data to Ble from MCU will
 * 					take place through using of FreeRtos API.				   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Bt_SendMessage(int8_t *pMsg, uint32_t iLen);

#ifdef __cplusplus
}
#endif


#endif /* CLUSTER_CONTROLLER_HAL_BT_H_ */
