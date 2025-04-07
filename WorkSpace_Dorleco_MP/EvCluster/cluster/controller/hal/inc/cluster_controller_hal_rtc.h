#ifndef CLUSTER_CONTROLLER_HAL_RTC_H_
#define CLUSTER_CONTROLLER_HAL_RTC_H_


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
 *	File Name	:	cluster_hal_i2c.h 								  	   		   
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
  * Function	   : Cluster_Controller_Hal_Rtc_Init							*
  * Params	       : void				        		                        *
  * Return value   : On success return BCU_OK else return BCU_NOK               *
  * Description	   : In this Method, Initialization and configuration with respe
  * 				 -ct to RTC module will take place including the RTC reset,
  * 				 And if the RTC module has battery support, it will skip the
  * 				 rtc reset and calls the RTC_Engine.					    *
  *-----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_Rtc_Init(void);

 /*-----------------------------------------------------------------------------*
  * Function	  :	Cluster_Controller_Hal_Rtc_DeInit						    *
  * Params	      :	void				        		                        *
  * Return value  :	On success return BCU_OK else return BCU_NOK			    *
  * Description	  :	In this Method, Denitialization of RTC will take place alo
  * 				-ng with deletion queue and thread whith is created during
  * 				the initialization.					                        *
  *-----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_Rtc_DeInit(void);

 /*----------------------------------------------------------------------------*
  * Function	  :	Cluster_Controller_Hal_Rtc_Enable						   *
  * Params	      :	int32_t iEnable			        		                   *
  * Return value  :	On success return BCU_OK else return BCU_NOK		       *
  * Description	  :	In this method, Enable and disable of RTC module will be
  * 				    take place.								               *
  *----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_Rtc_Enable(int32_t iEnable);

 /*-----------------------------------------------------------------------------*
  * Function	 :	Cluster_Controller_Hal_Rtc_SetReceiveQueueHandle		   *
  * Params	     :	QueueHandle_t  hHmiQueue				        		   *
  * Return value :	On success return BCU_OK else return BCU_NOK			   *
  * Description	 :	In this method, accessing the Hmi queue Id is take place
 					to send the parsed data.						           *
  *-----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_Rtc_SetReceiveQueueHandle(QueueHandle_t  hRtcQueue);

 /*-----------------------------------------------------------------------------*
  * Function	  :	Cluster_Controller_Hal_Rtc_GetDateTime					    *
  * Params	      :	uint8_t* pBuf, uint8_t iSize				                *
  * Return value  :	On success return BCU_OK else return BCU_NOK			    *
  * Description	  :   In this method, gets the data from RTC module via i2c.    *
  *-----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_Rtc_GetDateTime(sTimeInfo_t * pInfo);
 /*-----------------------------------------------------------------------------*
  * Function	  :	Cluster_Controller_Hal_Rtc_SetDateTime					    *
  * Params	      :	uint8_t* pBuf, uint8_t iSize				        	    *
  * Return value  :	On success return BCU_OK else return BCU_NOK		        *
  * Description	  :	In this method, The Api is exposed to CAN to set the data
  * 				and get the data from the rtc, and also if from the any mo
  * 				-dule wants change the time, can use this method.	        *
  *-----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_Rtc_SetDateTime(sTimeInfo_t * pInfo);
 /*-----------------------------------------------------------------------------*
  * Function	  :	Cluster_Controller_Hal_Rtc_UpdateDateTime					    *
  * Params	      :	uint8_t* pBuf, uint8_t iSize				        	    *
  * Return value  :	On success return BCU_OK else return BCU_NOK		        *
  * Description	  :		        *
  *-----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_Rtc_UpdateDateTime(void);
 /*-----------------------------------------------------------------------------*
  * Function	  :	Cluster_Controller_Hal_Rtc_SetMode   					    *
  * Params	      :	                            				        	    *
  * Return value  :	On success return BCU_OK else return BCU_NOK		        *
  * Description	  :		                                                        *
  *-----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_Rtc_SetMode(bool bMode);

#ifdef __cplusplus
}
#endif


#endif /* CLUSTER_CONTROLLER_HAL_RTC_H_ */
