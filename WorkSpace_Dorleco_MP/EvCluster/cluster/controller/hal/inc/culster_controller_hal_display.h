/*-----------------------------------------------------------------------------*
 *	OptM Media Solutions Confidential										   *
 *	Copyright (C) OptM Media Solutions Pvt. Ltd - All Rights Reserved	   	   *
 *																			   *
 *  Dissemination of this information or reproduction or redistribution of 	   *
 *  this material is  strictly forbidden unless prior written permission is	   *
 *  obtained from OptM Media Solutions Pvt Ltd								   *
 *	File Name	:	Winstar_WF70A8SYAHMNN0.h
 *  version		: 											              	   *
 *  Date		:	03-Mar-2023
 *  Description :  				 										 	   *
 *                               						                       *
 *																			   *
 *-----------------------------------------------------------------------------*/

#ifndef CLUSTER_CONTROLLER_HAL_DISPLAY_H_
#define CLUSTER_CONTROLLER_HAL_DISPLAY_H_

#include "fsl_display.h"
#include "fsl_mipi_dsi_cmd.h"

typedef struct _optm_resource
{
    mipi_dsi_device_t *dsiDevice;      /*!< MIPI DSI device. */
    void (*pullResetPin)(bool pullUp); /*!< Function to pull reset pin high or low. */
    void (*pullPowerPin)(bool pullup); /*!< Function to pull power pin high or low. */
} display_resource_t;

extern const display_operations_t optm_ops;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
status_t Cluster_Controller_Hal_Display_Init(display_handle_t *handle, const display_config_t *config);
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
status_t Cluster_Controller_Hal_Display_Deinit(display_handle_t *handle);
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
status_t Cluster_Controller_Hal_Display_Start(display_handle_t *handle);
/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
status_t Cluster_Controller_Hal_Display_Stop(display_handle_t *handle);
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Display_Enable(bool bOnOff);
#if defined(__cplusplus)
}
#endif

#endif /* CLUSTER_CONTROLLER_HAL_DISPLAY_H_ */
