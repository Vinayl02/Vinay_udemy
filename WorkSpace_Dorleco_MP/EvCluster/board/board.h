/*-----------------------------------------------------------------------------*
 *	OptM Media Solutions Confidential										   *
 *	Copyright (C) OptM Media Solutions Pvt Ltd - All Rights Reserved.	   	   *
 *  Dissemination of this information or reproduction or redistribution of 	   *
 *  this material is  strictly forbidden unless prior written permission is	   *
 *  obtained from OptM Media Solutions Pvt Ltd								   *
 *	File Name	:	board.h 							   *
 *  version		: 											              	   *
 *  Date		:												   *
 *  Description :  				 										 	   *
 *                               						                       *
 *-----------------------------------------------------------------------------*/

#ifndef _BOARD_H_
#define _BOARD_H_

#include "clock_config.h"
#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief The board name */
#define BOARD_NAME  "OPTM_CLUSTER"

/* The UART to use for debug messages. */
#define BOARD_DEBUG_UART_TYPE     kSerialPort_Uart
#define BOARD_DEBUG_UART_CLK_FREQ 24000000

/* Debug console */
#define BOARD_DEBUG_UART_BASEADDR (uint32_t) LPUART1
#define BOARD_DEBUG_UART_INSTANCE 1U
#define BOARD_UART_IRQ            LPUART1_IRQn
#define BOARD_UART_IRQ_HANDLER    LPUART1_IRQHandler


#ifndef BOARD_DEBUG_UART_BAUDRATE
#define BOARD_DEBUG_UART_BAUDRATE (115200U)
#endif /* BOARD_DEBUG_UART_BAUDRATE */

/* Definitions for eRPC MU transport layer */
#if defined(FSL_FEATURE_MU_SIDE_A)
#define MU_BASE        MUA
#define MU_IRQ         MUA_IRQn
#define MU_IRQ_HANDLER MUA_IRQHandler
#endif
#if defined(FSL_FEATURE_MU_SIDE_B)
#define MU_BASE        MUB
#define MU_IRQ         MUB_IRQn
#define MU_IRQ_HANDLER MUB_IRQHandler
#endif
#define MU_IRQ_PRIORITY (2)


/*! @brief The board flash size */
#define BOARD_FLASH_SIZE (0x1000000U)

/* SKIP_SEMC_INIT can also be defined independently */
#ifdef USE_SDRAM
#define SKIP_SEMC_INIT
#endif


/*! @brief The MIPI panel pins. */

#if(HW_REV_VERSION == 1)

/* Reset pin. */
#define CLUSTER_DISPLAY_RST_BANK    IOMUXC_GPIO_AD_07_GPIO9_IO06
#define CLUSTER_DISPLAY_RST_GPIO    GPIO9
#define CLUSTER_DISPLAY_RST_PIN     6
/* Driver pin. */
#define CLUSTER_DISPLAY_DRIVER_BANK IOMUXC_GPIO_AD_26_GPIO9_IO25
#define CLUSTER_DISPLAY_DRIVER_GPIO GPIO9
#define CLUSTER_DISPLAY_DRIVER_PIN  25

/* Back light pin. */
#if ENABLE_PWM_CONTROL
#define CLUSTER_DISPLAY_BL_BANK 	IOMUXC_GPIO_AD_27_FLEXPWM2_PWM1_B
#else
#define CLUSTER_DISPLAY_BL_BANK 	IOMUXC_GPIO_AD_27_GPIO9_IO26
#endif
#define CLUSTER_DISPLAY_BL_GPIO 	GPIO9
#define CLUSTER_DISPLAY_BL_PIN  	26


/* STANDBY pin. */
#define CLUSTER_DISPLAY_STB_BANK  IOMUXC_GPIO_AD_08_GPIO9_IO07
#define CLUSTER_DISPLAY_STB_GPIO GPIO9
#define CLUSTER_DISPLAY_STB_PIN  7

#else  //Hardware Version -2
/* Reset pin. */
#define CLUSTER_DISPLAY_RST_BANK   IOMUXC_GPIO_AD_07_GPIO9_IO06
#define CLUSTER_DISPLAY_RST_GPIO   GPIO9
#define CLUSTER_DISPLAY_RST_PIN    6
/* Driver pin. */
#define CLUSTER_DISPLAY_DRIVER_BANK IOMUXC_GPIO_AD_09_GPIO9_IO08
#define CLUSTER_DISPLAY_DRIVER_GPIO GPIO9
#define CLUSTER_DISPLAY_DRIVER_PIN  8

/* Back light pin. */
#if ENABLE_PWM_CONTROL
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_05_FLEXPWM1_PWM2_B,0U);
#else
#define CLUSTER_DISPLAY_BL_BANK IOMUXC_GPIO_AD_05_GPIO9_IO04
#endif

#define CLUSTER_DISPLAY_BL_GPIO GPIO9
#define CLUSTER_DISPLAY_BL_PIN  4

/* STANDBY pin. */
#define CLUSTER_DISPLAY_STB_BANK  IOMUXC_GPIO_AD_08_GPIO9_IO07
#define CLUSTER_DISPLAY_STB_GPIO  GPIO9
#define CLUSTER_DISPLAY_STB_PIN   7
#endif
/* Timer Manager definition. */
#define BOARD_TM_INSTANCE   1
#define BOARD_TM_CLOCK_ROOT kCLOCK_Root_Gpt1

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
 * API
 ******************************************************************************/
//uint32_t BOARD_DebugConsoleSrcFreq(void);

void BOARD_InitDebugConsole(void);

void BOARD_ConfigMPU(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_H_ */
