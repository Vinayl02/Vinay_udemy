/*-----------------------------------------------------------------------------*
 *	OptM Media Solutions Confidential										   *
 *	Copyright (C) OptM Media Solutions Pvt Ltd - All Rights Reserved.	   	   *
 *  Dissemination of this information or reproduction or redistribution of 	   *
 *  this material is  strictly forbidden unless prior written permission is	   *
 *  obtained from OptM Media Solutions Pvt Ltd								   *
 *	File Name	:	platform_display.h 							   *
 *  version		: 											              	   *
 *  Date		:												   *
 *  Description :  				 										 	   *
 *                               						                       *
 *-----------------------------------------------------------------------------*/
#pragma once

#include <platforminterface/screen.h>

#include "fsl_dc_fb.h"
#include "fsl_mipi_dsi.h"

#include "platform_config.h"

/* Where the frame buffer is shown in the screen. */
#define DEMO_BUFFER_START_X 0U
#define DEMO_BUFFER_START_Y 0U

#define CLUSTER_LCDIFV2 LCDIFV2

#define CLUSTER_DISPLAY_CONTROLLER_LCDIFV2 1


#if CLUSTER_DISPLAY_PANEL5

#define CLUSTER_PANEL_HEIGHT 480
#define CLUSTER_PANEL_WIDTH 800

#define LCDIF_CLK_DIV	20

#define CLUSTER_HSW	 2
#define CLUSTER_HFP	 44
#define CLUSTER_HBP	 16
#define CLUSTER_VSW	 2
#define CLUSTER_VFP	 43
#define CLUSTER_VBP	 5

#else

#define CLUSTER_PANEL_HEIGHT 600
#define CLUSTER_PANEL_WIDTH 1024

#if(CLUSTER_DISPLAY == 1)
#define LCDIF_CLK_DIV	13

#define CLUSTER_HSW	 70
#define CLUSTER_HFP	 60
#define CLUSTER_HBP	 160

#define CLUSTER_VSW	 20
#define CLUSTER_VFP	 7
#define CLUSTER_VBP	 23

#else
#define LCDIF_CLK_DIV	10
#define CLUSTER_HSW	 70
#define CLUSTER_HFP	 160
#define CLUSTER_HBP	 160
#define CLUSTER_VSW	 10
#define CLUSTER_VFP	 12
#define CLUSTER_VBP	 23
#endif

#endif

#if CLUSTER_DISPLAY_RGB888
#define VG_LITE_PIXEL_FORMAT VG_LITE_RGBA8888
#else
#define VG_LITE_PIXEL_FORMAT VG_LITE_BGRA8888
#endif




#define DEMO_POL_FLAGS \
    (kLCDIFV2_DataEnableActiveHigh | kLCDIFV2_VsyncActiveLow | kLCDIFV2_HsyncActiveLow \
     | kLCDIFV2_DriveDataOnFallingClkEdge)

#define DEMO_PANEL_RK055MHD091 MIPI_PANEL_RK055MHD091

#ifndef CLUSTER_PANEL
#define CLUSTER_PANEL USE_MIPI_PANEL
#endif

#ifndef DEMO_DISPLAY_CONTROLLER
/* Use LCDIFV2 by default, could use ELCDIF by changing this macro. */
#define CLUSTER_DISPLAY_CONTROLLER CLUSTER_DISPLAY_CONTROLLER_LCDIFV2            //
#endif

#define CLUSTER_BUFFER_WIDTH CLUSTER_PANEL_WIDTH
#define CLUSTER_BUFFER_HEIGHT CLUSTER_PANEL_HEIGHT

/*
 * The DPHY bit clock must be fast enough to send out the pixels, it should be
 * larger than:
 *
 *         (Pixel clock * bit per output pixel) / number of MIPI data lane
 *
 * Here the desired DPHY bit clock multiplied by ( 9 / 8 = 1.125) to ensure
 * it is fast enough.
 */
//#define DEMO_MIPI_DPHY_BIT_CLK_ENLARGE(origin) (((origin) / 8) * 9)

extern const dc_fb_t g_dc;
extern MIPI_DSI_Type g_mipiDsi;
#define CLUSTER_MIPI_DSI (&g_mipiDsi)
#define CLUSTER_MIPI_DSI_LANE_NUM 2

namespace Qul {
namespace Platform {
namespace Private {

void prepareDisplayController(void);
PlatformInterface::Screen *availableScreens(size_t *screenCount);

} // namespace Private
} // namespace Platform
} // namespace Qul
