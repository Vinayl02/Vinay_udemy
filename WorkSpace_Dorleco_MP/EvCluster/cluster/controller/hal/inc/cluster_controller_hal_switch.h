#ifndef CLUSTER_CONTROLLER_HAL_SWITCH_H_
#define CLUSTER_CONTROLLER_HAL_SWITCH_H_


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
 *	File Name	:	cluster_hal_gpio.h 								  	   		   
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
 int32_t Cluster_Controller_Hal_Switch_Init(void);
 /*-----------------------------------------------------------------------------*
  * Function	    :	Bcu_Cluster_Hal_Led_DeInit						   	       *
  * Params	    :					        		                           *
  * Return value	:	BCU_OK or BCU_NOK				                           *
  * Description	:							                                   *
  *-----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_Switch_DeInit(void);
 /*-----------------------------------------------------------------------------*
  * Function	    :	Bcu_Cluster_Hal_Led_Enable						   	       *
  * Params	    :					        		                           *
  * Return value	:	BCU_OK or BCU_NOK				                           *
  * Description	:							                                   *
  *-----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_Switch_SetReceiveQueueHandle(QueueHandle_t sQueueId);

 
#ifdef __cplusplus
}
#endif


#endif /* CLUSTER_CONTROLLER_HAL_LED_H_ */
