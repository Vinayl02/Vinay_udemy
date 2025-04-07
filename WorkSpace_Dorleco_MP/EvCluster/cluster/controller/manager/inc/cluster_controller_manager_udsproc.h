#ifndef CLUSTER_CONTROLLER_MANAGER_UDSPROC_H_
#define CLUSTER_CONTROLLER_MANAGER_UDSPROC_H_


#ifdef __cplusplus
 extern "C" {
#endif

/*-----------------------------------------------------------------------------*
 *	Optm Media Solutions Confidential										   *
 *	Copyright (C) Optm Media Solutions Pvt. Ltd - All Rights Reserved	   	   *
 *																			   *
 *  Dissemination of this information or reproduction or redistribution of 	   *
 *  this material is  strictly forbidden unless prior written permission is	   *
 *  obtained from Optm Media Solutions Pvt Ltd								   *
 *	File Name	:	cluster_hal_udsproc.h
 *  version		: 											              	   *
 *  Date		:	03-Mar-2023
 *  Description :  				 										 	   *
 *                               						                       *
 *																			   *
 *-----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*
 * Methods                      				                              *
 *----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Manager_UdsProc_Init();
 /*----------------------------------------------------------------------------*
  * Function	    :										        	   	      *
  * Params	    :				        		                          	  *
  * Return value	:									                          *
  * Description	:									                          *
  *----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Manager_UdsProc_DeInit(void);

 /*----------------------------------------------------------------------------*
  * Function	    :										        	   	      *
  * Params	    :				        		                          	  *
  * Return value	:									                          *
  * Description	:									                          *
  *----------------------------------------------------------------------------*/
 //int32_t Cluster_Controller_Manager_UdsProc_ServiceRequest(flexcan_frame_t* pFrame);

#ifdef __cplusplus
}
#endif


#endif /* CLUSTER_CONTROLLER_MANAGER_BTPROC_H_ */
