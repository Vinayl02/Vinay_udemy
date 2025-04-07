#ifndef CLUSTER_CONTROLLER_HAL_BACKLIGHT_H_
#define CLUSTER_CONTROLLER_HAL_BACKLIGHT_H_


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
 *	File Name	:	cluster_controller_hal_backlight.h 								  	   		   
 *  version		: 											              	   *
 *  Date		:	13-Mar-2023													   
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
* Function	    :											        		   *
* Params	    :					        		                           *
* Return value	:									                           *
* Description	:							                                   *
*-----------------------------------------------------------------------------*/

  int32_t Cluster_Controller_Hal_Backlight_Init(void);

/*-----------------------------------------------------------------------------*
* Function	    :											        		   *
* Params	    :					        		                           *
* Return value	:									                           *
* Description	:							                                   *
*-----------------------------------------------------------------------------*/

  int32_t Cluster_Controller_Hal_Backlight_DeInit(void);

/*-----------------------------------------------------------------------------*
* Function	    :											        		   *
* Params	    :					        		                           *
* Return value	:									                           *
* Description	:							                                   *
*-----------------------------------------------------------------------------*/

  int32_t Cluster_Controller_Hal_Backlight_SetBrightness(uint8_t iValue);

/*-----------------------------------------------------------------------------*
* Function	    :											        		   *
* Params	    :					        		                           *
* Return value	:									                           *
* Description	:							                                   *
*-----------------------------------------------------------------------------*/

  int32_t Cluster_Controller_Hal_Backlight_SetBrightnessLevel(uint32_t iLevel);



 /*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/

int32_t Cluster_Controller_Hal_Backlight_Enable(uint8_t iEnable);
#ifdef __cplusplus
}
#endif


#endif /* CLUSTER_CONTROLLER_HAL_BACKLIGHT_H_ */
