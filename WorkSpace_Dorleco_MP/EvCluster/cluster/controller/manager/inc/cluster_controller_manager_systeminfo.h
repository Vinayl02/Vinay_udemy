/*
 * cluster_controller_manager_systeminfo.h
 *
 *  Created on: Jul 6, 2023
 *      Author: Admin
 */

#ifndef CLUSTER_CONTROLLER_MANAGER_SYSTEMINFO_H_
#define CLUSTER_CONTROLLER_MANAGER_SYSTEMINFO_H_


 /*-----------------------------------------------------------------------------*
  * Function	    : 	Cluster_Controller_Manager_SystemInfo_Init				*
  * Params	        :	void			        		                        *
  * Return value	:	On success return BCU_OK else return BCU_NOK            *
  * Description  	:	In this function initializes the systeminfo and creates
  * 					xSemaphoreCreateBinary to synchronize the read and write
  * 					operation. And Read the data from secondary flash for re
  * 					-ference						                        *
  *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_SystemInfo_Init(void);
/*-----------------------------------------------------------------------------*
  * Function	    : 	Cluster_Controller_Manager_SystemInfo_DeInit			*
  * Params	        :	void			        		                        *
  * Return value	:	On success return BCU_OK else return BCU_NOK            *
  * Description  	:	In this function Deinitializes the systeminfo and delete
  * 					vSemaphore which is created in initialization.			*
  *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_SystemInfo_DeInit(void);
/*-----------------------------------------------------------------------------*
  * Function	    : 	Cluster_Controller_UpdateSystemInfo						*
  * Params	        :	ClusterInfo_t* pInfo and int32_t iValue			       	*
  * Return value	:	On success return BCU_OK else return BCU_NOK            *
  * Description  	:	In this function,Write the data to secondary flash(Nvm) *
  *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_UpdateSystemInfo(ClusterInfo_t* pInfo, int32_t iValue);
/*-----------------------------------------------------------------------------*
 * Function	    : 	Cluster_Controller_GetSystemInfo						*
 * Params	        :	ClusterInfo_t* pInfo		       						*
 * Return value	:	On success return BCU_OK else return BCU_NOK            *
 * Description  	:	In this function,Read the data to secondary flash(Nvm)  *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_GetSystemInfo(ClusterInfo_t* pInfo);
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_GetVehicleSpecInfo(sDeviceInfo_t * pInfo);
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t SystemInfo_UpdateProductInfo(uint64_t iInfo, uint32_t iSectorAddress);
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_ResetSystemInfo(ClusterInfo_t* pInfo);
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_SetVehicleSpecInfo(void * pInfo, int32_t iParam);
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t  Cluster_Controller_GetDTCInfo(uint8_t * pDtcInfo);
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Manager_SystemInfo_GetCache(void);

#endif /* CLUSTER_CONTROLLER_MANAGER_SYSTEMINFO_H_ */
