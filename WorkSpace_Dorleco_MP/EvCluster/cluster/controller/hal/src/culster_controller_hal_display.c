/*-----------------------------------------------------------------------------*
 *	OptM Media Solutions Confidential										   *
 *	Copyright (C) OptM Media Solutions Pvt Ltd - All Rights Reserved.	   	   *
 *  Dissemination of this information or reproduction or redistribution of 	   *
 *  this material is  strictly forbidden unless prior written permission is	   *
 *  obtained from OptM Media Solutions Pvt Ltd								   *
 *	File Name	:	Fsl_Display_OptM_Custom.c 							   *
 *  version		: 											              	   *
 *  Date		:	03-Mar-2023												   *
 *  Description :  				 										 	   *
 *                               						                       *
 *-----------------------------------------------------------------------------*/
#include <culster_controller_hal_display.h>
#include "fsl_display.h"
#include "fsl_gpio.h"
#include "board.h"
#include "fsl_debug_console.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define Display_DelayMs VIDEO_DelayMs

/*******************************************************************************
 * Variables
 ******************************************************************************/
const display_operations_t optm_ops = {
    .init   = Cluster_Controller_Hal_Display_Init,
    .deinit = Cluster_Controller_Hal_Display_Deinit,
    .start  = Cluster_Controller_Hal_Display_Start,
    .stop   = Cluster_Controller_Hal_Display_Stop,
};


typedef struct
{
	bool				bIsOn;
	int32_t 			iIsInitialised;
	display_handle_t * pHandle;
	mipi_dsi_device_t *pDsiDevice;
}HalDisplay_t;

static HalDisplay_t   hDisplay= {0};

/*----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Display_Init									        	   	      *
 * Params	    :	display_handle_t *handle, const display_config_t *config			        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
status_t Cluster_Controller_Hal_Display_Init(display_handle_t *handle, const display_config_t *config)
{
    status_t status                   = kStatus_Success;
    if((handle != NULL) && (config!= NULL))
    {
		const display_resource_t *resource = (const display_resource_t *)(handle->resource);

		memset(&hDisplay, 0 ,sizeof(HalDisplay_t));

		hDisplay.pHandle = handle;
		hDisplay.pDsiDevice = resource->dsiDevice;

		if (config->resolution == FSL_VIDEO_RESOLUTION(1024, 600))
		{
			Display_DelayMs(100);

#if CLUSTER_KEYIGNITION_SUPPORTED
			status = Cluster_Controller_Hal_Display_Enable(0);
#else
			status = Cluster_Controller_Hal_Display_Enable(1);
#endif

			hDisplay.iIsInitialised = 1;
		}
		else
		{
			status= kStatus_InvalidArgument;
		}
    }
    return status;
}
/*----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Display_Deinit									        	   	      *
 * Params	    :	display_handle_t *handle			        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
status_t Cluster_Controller_Hal_Display_Deinit(display_handle_t *handle)
{
    const display_resource_t *resource = (const display_resource_t *)(handle->resource);
    mipi_dsi_device_t *dsiDevice      = resource->dsiDevice;

    (void)MIPI_DSI_DCS_EnterSleepMode(dsiDevice, true);

    resource->pullResetPin(false);
    resource->pullPowerPin(false);

    return kStatus_Success;
}
/*----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Display_Start									        	   	      *
 * Params	    :	display_handle_t *handle			        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
status_t Cluster_Controller_Hal_Display_Start(display_handle_t *handle)
{

    status_t status                   = kStatus_Success;
    const display_resource_t *resource = (const display_resource_t *)(handle->resource);
    mipi_dsi_device_t *dsiDevice      = resource->dsiDevice;

    if(hDisplay.bIsOn)
    {
    	status = MIPI_DSI_DCS_SetDisplayOn(dsiDevice, true);
    }
    return status;
}

/*----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Display_Start									        	   	      *
 * Params	    :	display_handle_t *handle			        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
status_t Cluster_Controller_Hal_Display_Enable( bool bOnOff)
{
	status_t status = kStatus_Success;
	if((hDisplay.pHandle != NULL) &&
	   (hDisplay.pDsiDevice != NULL))
	{
		hDisplay.bIsOn = bOnOff;
		if(bOnOff)
		{
			GPIO_PinWrite(CLUSTER_DISPLAY_RST_GPIO, CLUSTER_DISPLAY_RST_PIN, 0);    		 //Reset
			Display_DelayMs(20);
			GPIO_PinWrite(CLUSTER_DISPLAY_DRIVER_GPIO, CLUSTER_DISPLAY_DRIVER_PIN, bOnOff);      	 //display driver
			Display_DelayMs(20);

			GPIO_PinWrite(CLUSTER_DISPLAY_STB_GPIO, CLUSTER_DISPLAY_STB_PIN, 0);   		 //standby
			Display_DelayMs(5);

			GPIO_PinWrite(CLUSTER_DISPLAY_RST_GPIO, CLUSTER_DISPLAY_RST_PIN, 1);    		 //Reset
			Display_DelayMs(20);
			GPIO_PinWrite(CLUSTER_DISPLAY_RST_GPIO, CLUSTER_DISPLAY_RST_PIN, 0);    		 //Reset
			Display_DelayMs(20);

			GPIO_PinWrite(CLUSTER_DISPLAY_RST_GPIO, CLUSTER_DISPLAY_RST_PIN, 1);    		 //Reset
			Display_DelayMs(20);

			GPIO_PinWrite(CLUSTER_DISPLAY_RST_GPIO, CLUSTER_DISPLAY_RST_PIN, 0);    		 //Reset
			Display_DelayMs(100);

			status = MIPI_DSI_GenericWrite(hDisplay.pDsiDevice, (const uint8_t[]){0xB2, 0x10}, 2); // 2 lane and Black/White background

			status = MIPI_DSI_GenericWrite(hDisplay.pDsiDevice, (const uint8_t[]){0x80, 0x5b}, 2); // Gamma

			status = MIPI_DSI_GenericWrite(hDisplay.pDsiDevice, (const uint8_t[]){0x81, 0x47}, 2); // Gamma

			status = MIPI_DSI_GenericWrite(hDisplay.pDsiDevice, (const uint8_t[]){0x82, 0x84}, 2); // Gamma

			status = MIPI_DSI_GenericWrite(hDisplay.pDsiDevice, (const uint8_t[]){0x83, 0x88}, 2); // Gamma

			status = MIPI_DSI_GenericWrite(hDisplay.pDsiDevice, (const uint8_t[]){0x84, 0x88}, 2); // Gamma

			status = MIPI_DSI_GenericWrite(hDisplay.pDsiDevice, (const uint8_t[]){0x85, 0x23}, 2); // Gamma

			status = MIPI_DSI_GenericWrite(hDisplay.pDsiDevice, (const uint8_t[]){0x86, 0xb6}, 2); // Gamma



			status =MIPI_DSI_DCS_EnterSleepMode(hDisplay.pDsiDevice, false);

			Display_DelayMs(20);
			GPIO_PinWrite(CLUSTER_DISPLAY_BL_GPIO, CLUSTER_DISPLAY_BL_PIN, 1);      	 //back light


			status = Cluster_Controller_Hal_Display_Start(hDisplay.pHandle);
		}
		else
		{
			status = Cluster_Controller_Hal_Display_Stop(hDisplay.pHandle);

			GPIO_PinWrite(CLUSTER_DISPLAY_BL_GPIO, CLUSTER_DISPLAY_BL_PIN, 0);
			Display_DelayMs(20);

			status = MIPI_DSI_DCS_EnterSleepMode(hDisplay.pDsiDevice, true);

			GPIO_PinWrite(CLUSTER_DISPLAY_RST_GPIO, CLUSTER_DISPLAY_RST_PIN, 1);    		 //Reset
			Display_DelayMs(20);

			GPIO_PinWrite(CLUSTER_DISPLAY_STB_GPIO, CLUSTER_DISPLAY_STB_PIN, 1);   		 //standby
			Display_DelayMs(20);

			GPIO_PinWrite(CLUSTER_DISPLAY_DRIVER_GPIO, CLUSTER_DISPLAY_DRIVER_PIN, 0);      	 //display driver
			Display_DelayMs(20);
		}
	}
	else
	{
		PRINTF("\n\r Cluster_Controller_Hal_Display_Enable Invalid State \n\r");
	}
	return status;
}
/*----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Display_Stop									        	   	      *
 * Params	    :	display_handle_t *handle		        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
status_t Cluster_Controller_Hal_Display_Stop(display_handle_t *handle)
{
    const display_resource_t *resource = (const display_resource_t *)(handle->resource);
    mipi_dsi_device_t *dsiDevice      = resource->dsiDevice;

    return MIPI_DSI_DCS_SetDisplayOn(dsiDevice, false);
}
