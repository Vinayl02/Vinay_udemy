/*
 * cluster_controller_manager_rtcproc.h
 *
 *  Created on: Feb 27, 2024
 *      Author: Admin
 */

#ifndef CLUSTER_CONTROLLER_MANAGER_RTCPROC_H_
#define CLUSTER_CONTROLLER_MANAGER_RTCPROC_H_
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
  *	File Name	:	cluster_controller_manager_rtcproc.h
  *  version		: 											              	   *
  *  Date		:	03-Mar-2023
  *  Description :  				 										 	   *
  *                               						                       *
  *																			   *
  *-----------------------------------------------------------------------------*/

 /*----------------------------------------------------------------------------*
  * Function	    :										        	   	      *
  * Params	    :				        		                          	  *
  * Return value	:									                          *
  * Description	:									                          *
  *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_RtcProc_Init(QueueHandle_t RtcQueueId);
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_RtcProc_DeInit(void);
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_RtcProc_GetDateTime(sTimeInfo_t * pInfo);
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_RtcProc_SetDateTime(uint8_t* pBuf, uint8_t iSize);
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_RtcProc_CheckForValidData(sTimeInfo_t * pRTCInfo );
#ifdef __cplusplus
}
#endif

#endif /* CLUSTER_CONTROLLER_MANAGER_RTCPROC_H_ */
