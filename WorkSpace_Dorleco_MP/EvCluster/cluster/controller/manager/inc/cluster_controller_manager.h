#ifndef CLUSTER_CONTROLLER_MANAGER_H_
#define CLUSTER_CONTROLLER_MANAGER_H_


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
 *	File Name	:	cluster_manager.h 								  	   		   
 *  version		: 											              	   *
 *  Date		:	03-Mar-2023													   
 *  Description :  				 										 	   * 
 *                               						                       *
 *																			   *
 *-----------------------------------------------------------------------------*/
 
/*----------------------------------------------------------------------------* 
* Include Headers				                         				      * 
*-----------------------------------------------------------------------------*/
#include <stdio.h>
#include "cluster_common.h"
#include "FreeRTOS.h"
#include "queue.h"

/*----------------------------------------------------------------------------* 
* Macro ,Structure and enum definations                   				      * 
*-----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------* 
* Methods  								                 				      * 
*-----------------------------------------------------------------------------*/
 
 

/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_Init(sController_t * pCtl);
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_DeInit(void);

/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_Start();
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_Stop(int32_t iModules);
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_Set_QueueHandle(int32_t iModules,QueueHandle_t iMsgQ);
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_RtcGetDateTime(sTimeInfo_t * pInfo);

/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_RtcSetDateTime(uint8_t* pBuf, uint8_t iSize);

/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/

int32_t  Cluster_Controller_Manager_Display_Set_BrightnessLevel(uint32_t iLevel);

/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_Display_Set_Brightness(uint8_t iValue);
#ifdef __cplusplus
 }
#endif


#endif /* CLUSTER_CONTROLLER_MANAGER_H_ */
