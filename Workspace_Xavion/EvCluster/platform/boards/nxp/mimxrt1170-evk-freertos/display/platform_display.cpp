/*-----------------------------------------------------------------------------*
 *	OptM Media Solutions Confidential										   *
 *	Copyright (C) OptM Media Solutions Pvt Ltd - All Rights Reserved.	   	   *
 *  Dissemination of this information or reproduction or redistribution of 	   *
 *  this material is  strictly forbidden unless prior written permission is	   *
 *  obtained from OptM Media Solutions Pvt Ltd								   *
 *	File Name	:	platform_display.cpp 							   *
 *  version		: 											              	   *
 *  Date		:												   *
 *  Description :  				 										 	   *
 *                               						                       *
 *-----------------------------------------------------------------------------*/
#include <culster_controller_hal_display.h>
#include "display/platform_display.h"

#include "fsl_gpio.h"
#include "fsl_mipi_dsi.h"
#include "pin_mux.h"
#include "board.h"
#include "fsl_debug_console.h"

#include "fsl_dc_fb_lcdifv2.h"

#include <platforminterface/log.h>
#include <qul/pixelformat.h>



#define CLUSTER_LCDIF_POL_FLAGS                                                             \
    (kLCDIFV2_DataEnableActiveHigh | kLCDIFV2_VsyncActiveLow | kLCDIFV2_HsyncActiveLow | \
     kLCDIFV2_DriveDataOnFallingClkEdge)

#define CLUSTER_LCDIF LCDIFV2


static dc_fb_lcdifv2_handle_t s_dcFbLcdifv2Handle = {0};

static const dc_fb_lcdifv2_config_t s_dcFbLcdifv2Config = {
    .lcdifv2       = CLUSTER_LCDIF,
    .width         = CLUSTER_PANEL_WIDTH,
    .height        = CLUSTER_PANEL_HEIGHT,
    .hsw           = CLUSTER_HSW,
    .hfp           = CLUSTER_HFP,
    .hbp           = CLUSTER_HBP,
    .vsw           = CLUSTER_VSW,
    .vfp           = CLUSTER_VFP,
    .vbp           = CLUSTER_VBP,
    .polarityFlags = CLUSTER_LCDIF_POL_FLAGS,
    .lineOrder     = kLCDIFV2_LineOrderRGB,
/* CM4 is domain 1, CM7 is domain 0. */
	.domain = 0,
};

const dc_fb_t g_dc = {
    .ops     = &g_dcFbOpsLcdifv2,
    .prvData = &s_dcFbLcdifv2Handle,
    .config  = &s_dcFbLcdifv2Config,
};

/*
 * The DPHY bit clock must be fast enough to send out the pixels, it should be
 * larger than:
 *
 *         (Pixel clock * bit per output pixel) / number of MIPI data lane
 *
 * Here the desired DPHY bit clock multiplied by ( 9 / 8 = 1.125) to ensure
 * it is fast enough.
 */

#define CLUSTER_MIPI_DPHY_BIT_CLK_ENLARGE(origin) (((origin) / 8) * 9)

namespace Qul {
namespace Platform {
namespace Private {

uint32_t mipiDsiTxEscClkFreq_Hz;
uint32_t mipiDsiDphyBitClkFreq_Hz;
uint32_t mipiDsiDphyRefClkFreq_Hz;
uint32_t mipiDsiDpiClkFreq_Hz;

MIPI_DSI_Type g_mipiDsi =
{
    .host = DSI_HOST,
    .apb = DSI_HOST_APB_PKT_IF,
    .dpi = DSI_HOST_DPI_INTFC,
    .dphy = DSI_HOST_DPHY_INTFC,
};

const Qul::PixelFormat kLcdFormat = Qul::PixelFormat_RGB32;
const uint16_t kLcdWidth = CLUSTER_PANEL_WIDTH;
const uint16_t kLcdHeight = CLUSTER_PANEL_HEIGHT;

static status_t verifyDisplayClockSource(void)
{
    status_t status;
    uint32_t srcClkFreq;

    /*
     * In this implementation, the SYSPLL2 (528M) clock is used as the source
     * of LCDIFV2 pixel clock and MIPI DSI ESC clock. The OSC24M clock is used
     * as the MIPI DSI DPHY PLL reference clock. This function checks the clock
     * source are valid. OSC24M is always valid, so only verify the SYSPLL2.
     */
    srcClkFreq = CLOCK_GetPllFreq(kCLOCK_PllSys2);
    if (528 != (srcClkFreq / 1000000))
    {
        status = kStatus_Fail;
    }
    else
    {
        status = kStatus_Success;
    }

    return status;
}

static void pullResetPin(bool pullUp)
{
    if (pullUp)
    {
        GPIO_PinWrite(CLUSTER_DISPLAY_RST_GPIO, CLUSTER_DISPLAY_RST_PIN, 1);
    }
    else
    {
        GPIO_PinWrite(CLUSTER_DISPLAY_RST_GPIO, CLUSTER_DISPLAY_RST_PIN, 0);
    }

}

/* From the schematic, the power pin is pinned to high. */
static void pullPowerPin(bool pullUp)
{

    if (pullUp)
    {
        GPIO_PinWrite(CLUSTER_DISPLAY_DRIVER_GPIO, CLUSTER_DISPLAY_DRIVER_PIN, 1);
    }
    else
    {
        GPIO_PinWrite(CLUSTER_DISPLAY_DRIVER_GPIO, CLUSTER_DISPLAY_DRIVER_PIN, 0);
    }
}


static status_t dsiTransfer(dsi_transfer_t *xfer)
{
    return DSI_TransferBlocking(CLUSTER_MIPI_DSI, xfer);
}

static mipi_dsi_device_t dsiDevice = {
    .virtualChannel = 0,
    .xferFunc       = dsiTransfer,
};

static const display_resource_t displayResource = {
    .dsiDevice    = &dsiDevice,
    .pullResetPin = pullResetPin,
	.pullPowerPin = pullPowerPin,
};

static display_handle_t optMDispHandle = {
    .resource = &displayResource,
    .ops      = &optm_ops,
};

static void initLcdifClock(void)
{
    /*
     * The pixel clock is (height + VSW + VFP + VBP) * (width + HSW + HFP + HBP) * frame rate.
     *
     * Use PLL_528 as clock source.
     *
     * For 60Hz frame rate, the RK055IQH091 pixel clock should be 36MHz.
     * the RK055AHD091 pixel clock should be 62MHz.
     */
    const clock_root_config_t lcdifv2ClockConfig = {
        .clockOff = false,
        .mux      = 4, /* !< PLL_528. */
        .div      = LCDIF_CLK_DIV,  //changed value from 9
    };

    CLOCK_SetRootClock(kCLOCK_Root_Lcdifv2, &lcdifv2ClockConfig);

    mipiDsiDpiClkFreq_Hz = CLOCK_GetRootClockFreq(kCLOCK_Root_Lcdifv2);
}

static status_t initLcdPanel(void)
{
    status_t status;

    const gpio_pin_config_t pinConfig = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

    const display_config_t displayConfig = {
        .resolution   = FSL_VIDEO_RESOLUTION(CLUSTER_PANEL_WIDTH, CLUSTER_PANEL_HEIGHT),
        .hsw          = CLUSTER_HSW,
        .hfp          = CLUSTER_HFP,
        .hbp          = CLUSTER_HBP,
        .vsw          = CLUSTER_VSW,
        .vfp          = CLUSTER_VFP,
        .vbp          = CLUSTER_VBP,
        .controlFlags = 0,
        .dsiLanes     = CLUSTER_MIPI_DSI_LANE_NUM,
    };

    GPIO_PinInit(CLUSTER_DISPLAY_DRIVER_GPIO, CLUSTER_DISPLAY_DRIVER_PIN, &pinConfig);
    GPIO_PinInit(CLUSTER_DISPLAY_BL_GPIO, CLUSTER_DISPLAY_BL_PIN, &pinConfig);
    GPIO_PinInit(CLUSTER_DISPLAY_RST_GPIO, CLUSTER_DISPLAY_RST_PIN, &pinConfig);
    GPIO_PinInit(CLUSTER_DISPLAY_STB_GPIO, CLUSTER_DISPLAY_STB_PIN, &pinConfig);
	
    status = Cluster_Controller_Hal_Display_Init(&optMDispHandle, &displayConfig);
    return status;
}

static void initMipiDsiClock(void)
{
    uint32_t mipiDsiEscClkFreq_Hz;

    /* RxClkEsc max 60MHz, TxClkEsc 12 to 20MHz. */
    /* RxClkEsc = 528MHz / 11 = 48MHz. */
    /* TxClkEsc = 528MHz / 11 / 4 = 16MHz. */
    const clock_root_config_t mipiEscClockConfig = {
        .clockOff = false,
        .mux      = 4, /* !< PLL_528. */
        .div      = LCDIF_CLK_DIV,//11,
    };

    CLOCK_SetRootClock(kCLOCK_Root_Mipi_Esc, &mipiEscClockConfig);

    mipiDsiEscClkFreq_Hz = CLOCK_GetRootClockFreq(kCLOCK_Root_Mipi_Esc);

    const clock_group_config_t mipiEscClockGroupConfig = {
        .clockOff = false,
        .resetDiv = 2,
        .div0     = 2, /* TX esc clock. */
    };

    CLOCK_SetGroupConfig(kCLOCK_Group_MipiDsi, &mipiEscClockGroupConfig);
#if(CLUSTER_DISPLAY == 1)
    mipiDsiTxEscClkFreq_Hz = mipiDsiEscClkFreq_Hz / 3;
#else
    mipiDsiTxEscClkFreq_Hz = mipiDsiEscClkFreq_Hz / 2;
#endif

    /* DPHY reference clock, use OSC 24MHz clock. */
    const clock_root_config_t mipiDphyRefClockConfig = {
        .clockOff = false,
        .mux      = 1, /* !< OSC_24M. */
        .div      = 1,
    };

    CLOCK_SetRootClock(kCLOCK_Root_Mipi_Ref, &mipiDphyRefClockConfig);

    mipiDsiDphyRefClkFreq_Hz = BOARD_XTAL0_CLK_HZ;
}

static void setMipiDsiConfig(void)
{
    dsi_config_t dsiConfig;
    dsi_dphy_config_t dphyConfig;

    const dsi_dpi_config_t dpiConfig = {.pixelPayloadSize = CLUSTER_PANEL_WIDTH,
                                        .dpiColorCoding   = kDSI_Dpi24Bit,
                                        .pixelPacket      = kDSI_PixelPacket24Bit,
                                        .videoMode        = kDSI_DpiBurst,
                                        .bllpMode         = kDSI_DpiBllpLowPower,
                                        .polarityFlags    = kDSI_DpiVsyncActiveLow | kDSI_DpiHsyncActiveLow,
                                        .hfp              = CLUSTER_HFP,
                                        .hbp              = CLUSTER_HBP,
                                        .hsw              = CLUSTER_HSW,
                                        .vfp              = CLUSTER_VFP,
                                        .vbp              = CLUSTER_VBP,
                                        .panelHeight      = CLUSTER_PANEL_HEIGHT,
                                        .virtualChannel   = 0};

    /*
     * dsiConfig.numLanes = 4;
     * dsiConfig.enableNonContinuousHsClk = false;
     * dsiConfig.autoInsertEoTp = true;
     * dsiConfig.numExtraEoTp = 0;
     * dsiConfig.htxTo_ByteClk = 0;
     * dsiConfig.lrxHostTo_ByteClk = 0;
     * dsiConfig.btaTo_ByteClk = 0;
     */
    DSI_GetDefaultConfig(&dsiConfig);
    dsiConfig.numLanes       = CLUSTER_MIPI_DSI_LANE_NUM;
    dsiConfig.autoInsertEoTp = true;

    /* Init the DSI module. */
    DSI_Init(CLUSTER_MIPI_DSI, &dsiConfig);

    /* Init DPHY.
     *
     * The DPHY bit clock must be fast enough to send out the pixels, it should be
     * larger than:
     *
     *         (Pixel clock * bit per output pixel) / number of MIPI data lane
     *
     * Here the desired DPHY bit clock multiplied by ( 9 / 8 = 1.125) to ensure
     * it is fast enough.
     *
     * Note that the DSI output pixel is 24bit per pixel.
     */
    mipiDsiDphyBitClkFreq_Hz = mipiDsiDpiClkFreq_Hz * (24 / CLUSTER_MIPI_DSI_LANE_NUM);

    mipiDsiDphyBitClkFreq_Hz = CLUSTER_MIPI_DPHY_BIT_CLK_ENLARGE(mipiDsiDphyBitClkFreq_Hz);

    DSI_GetDphyDefaultConfig(&dphyConfig, mipiDsiDphyBitClkFreq_Hz, mipiDsiTxEscClkFreq_Hz);

    mipiDsiDphyBitClkFreq_Hz = DSI_InitDphy(CLUSTER_MIPI_DSI, &dphyConfig, mipiDsiDphyRefClkFreq_Hz);

    /* Init DPI interface. */
    DSI_SetDpiConfig(CLUSTER_MIPI_DSI, &dpiConfig, CLUSTER_MIPI_DSI_LANE_NUM, mipiDsiDpiClkFreq_Hz, mipiDsiDphyBitClkFreq_Hz);
}

static status_t initDisplayInterface(void)
{
#if CLUSTER_DISPLAY_RGB888


	  /* GPIO configuration of BACKLIGHT on GPIO_AD_07 (pin T17) */
	  gpio_pin_config_t BACKLIGHT_config = {
	      .direction = kGPIO_DigitalOutput,
	      .outputLogic = 1U,
	      .interruptMode = kGPIO_NoIntmode
	  };
	  /* Initialize GPIO functionality on GPIO_AD_07 (pin T17) */
	  GPIO_PinInit(GPIO9, 6U, &BACKLIGHT_config);

	/* LCDIF v2 output to Parallel LCD. */
	CLOCK_EnableClock(kCLOCK_Video_Mux);
    VIDEO_MUX->VID_MUX_CTRL.SET = VIDEO_MUX_VID_MUX_CTRL_PARA_LCD_SEL_MASK;

    return kStatus_Success;

#else

	/* LCDIF v2 output to MIPI DSI. */
    CLOCK_EnableClock(kCLOCK_Video_Mux);
    VIDEO_MUX->VID_MUX_CTRL.SET = VIDEO_MUX_VID_MUX_CTRL_MIPI_DSI_SEL_MASK;

    /* 1. Power on and isolation off. */
    PGMC_BPC4->BPC_POWER_CTRL |= (PGMC_BPC_BPC_POWER_CTRL_PSW_ON_SOFT_MASK | PGMC_BPC_BPC_POWER_CTRL_ISO_OFF_SOFT_MASK);

    /* 2. Assert MIPI reset. */
    IOMUXC_GPR->GPR62 &=
        ~(IOMUXC_GPR_GPR62_MIPI_DSI_PCLK_SOFT_RESET_N_MASK | IOMUXC_GPR_GPR62_MIPI_DSI_ESC_SOFT_RESET_N_MASK |
          IOMUXC_GPR_GPR62_MIPI_DSI_BYTE_SOFT_RESET_N_MASK | IOMUXC_GPR_GPR62_MIPI_DSI_DPI_SOFT_RESET_N_MASK);

    /* 3. Setup clock. */
    initMipiDsiClock();

    /* 4. Deassert PCLK and ESC reset. */
    IOMUXC_GPR->GPR62 |=
        (IOMUXC_GPR_GPR62_MIPI_DSI_PCLK_SOFT_RESET_N_MASK | IOMUXC_GPR_GPR62_MIPI_DSI_ESC_SOFT_RESET_N_MASK);

    /* 5. Configures peripheral. */
    setMipiDsiConfig();

    /* 6. Deassert BYTE and DBI reset. */
    IOMUXC_GPR->GPR62 |=
        (IOMUXC_GPR_GPR62_MIPI_DSI_BYTE_SOFT_RESET_N_MASK | IOMUXC_GPR_GPR62_MIPI_DSI_DPI_SOFT_RESET_N_MASK);

    /* 7. Configure the panel. */
    return initLcdPanel();

#endif

}

void prepareDisplayController(void)
{
    QUL_ASSERT(verifyDisplayClockSource() == kStatus_Success, QulError_Display_InitializationFailed);

    initLcdifClock();

    QUL_ASSERT(initDisplayInterface() == kStatus_Success, QulError_Display_InitializationFailed);

    NVIC_ClearPendingIRQ(LCDIFv2_IRQn);
    NVIC_SetPriority(LCDIFv2_IRQn, 3);
    EnableIRQ(LCDIFv2_IRQn);

    QUL_ASSERT(g_dc.ops->init(&g_dc) == kStatus_Success, QulError_Display_InitializationFailed);
}

PlatformInterface::Screen *availableScreens(size_t *screenCount)
{
    *screenCount = 1;
    static PlatformInterface::Screen screen(PlatformInterface::Size(kLcdWidth, kLcdHeight), kLcdFormat);
    return &screen;
}

} // namespace Private
} // namespace Platform
} // namespace Qul
