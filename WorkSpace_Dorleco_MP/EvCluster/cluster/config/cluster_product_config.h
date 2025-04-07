#ifndef CLUSTER_PRODUCT_CONFIG_H_
#define CLUSTER_PRODUCT_CONFIG_H_


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
 *	File Name	:	cluster_product_config.h								   *
 *  version		: 											              	   *
 *  Date		:	03-Mar-2023												   *
 *  Description :  				 										 	   *
 *                               						                       *
 *																			   *
 *-----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*
 * Include Headers				           	                             	  *
 *----------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------*
 * Include Headers				           	                             	   *
 *-----------------------------------------------------------------------------*/

/* FreeRTOS kernel includes. */
#include <fsl_lpuart_freertos.h>
#include "FreeRTOS.h"


/* Freescale includes. */
#include "fsl_device_registers.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_common.h"
#include "fsl_lpuart.h"

/*----------------------------------------------------------------------------*
 * Structure and enum definitions				                               *
 *-----------------------------------------------------------------------------*/

 /*----------------------------------------------------------------------------*
 * CAN Interface 									                          *
 *----------------------------------------------------------------------------*/

#define CAN_INTERFACE_ID               	    3
#define	CAN_INTERFACE_BAUDRATE	    	    500000

#if(HW_REV_VERSION == 1)
#define 	CLUSTER_VERSION                "Cluster_Qucev_V6.9_06092024"
#define 	SOFTWARE_VERSION                69
#define 	HARDWARE_VERSION                10
#else
#define 	CLUSTER_VERSION                "Cluster_Qucev_MP_V9.53_04042025"  //V9.53
#define 	SOFTWARE_VERSION                952
#define 	HARDWARE_VERSION                200
#endif

#if(UDS_SERVICE_ENABLE == 1)
#define CLUSTER_RX_CANID_MAX			 	6
#else
#define CLUSTER_RX_CANID_MAX			 	5
#endif

static sCanConfig_t Cluster_Rx_CanId_List[CLUSTER_RX_CANID_MAX] =
{
	{eCANID_Frame1,0},
	{eCANID_Frame2,0},
	{eCANID_Frame7,0},
	{eCANID_Frame8,0},
	{eCANID_Internal,0}
#if(UDS_SERVICE_ENABLE == 1)
	,{eCANID_UDSRx,0}
#endif
};
/*----------------------------------------------------------------------------*
 * RTC Interface 									                          *
 *----------------------------------------------------------------------------*/
#define     RTC_INTERFACE_ID                1
#define		RTC_INTERFACE_BAUDRATE			100000
/*----------------------------------------------------------------------------*
 * UART Interface 									                          *
 *----------------------------------------------------------------------------*/
#define    	BT_INTERFACE_ID               	10
#define		BT_INTERFACE_BAUDRATE	    	115200


#define		RTC_INTERRUPT_CONFIGURATION		0

/*----------------------------------------------------------------------------*
 * Backlight Interface 									                      *
 *-----------------------------------------------------------------------------*/

#define	CLUSTER_BACKLIGHT_LEVEL_MAX			31
#define	CLUSTER_BACKLIGHT_LEVEL_DEFAULT		26


 static uint8_t  Backlight_BrightnessMap[CLUSTER_BACKLIGHT_LEVEL_MAX] =
 	 	 	 	 	 	 	 	 	 	{
 	 	 	 	 	 	 	 	 	 	  2, 3, 6, 9, 12, 15, 18, 21, 24, 27,
 	 	 	 	 	 	 	 	 	 	  30,33, 36, 39, 42, 45,48, 51, 54, 57,
										  60,63, 66, 69, 72, 75, 78, 81, 84, 87,
										  95
 	 	 	 	 	 	 	 	 	 	};

/*----------------------------------------------------------------------------*
 * PWM Interface 									                       	  *
 *-----------------------------------------------------------------------------*/
#define		BACKLIGHT_PWM_INTERFACE_ID					2
#define		BACKLIGHT_PWM_INTERFACE_FREQUENCE			(1000UL)


/*----------------------------------------------------------------------------*
 * Stack									                       	  *
 *-----------------------------------------------------------------------------*/
#define		PRODUCT_SUBMODULE_STACK_SIZE  		(configMINIMAL_STACK_SIZE * 2)

/*----------------------------------------------------------------------------*
 * SWITCH Interface 									                      *
 *-----------------------------------------------------------------------------*/

static int32_t GPIO_List[10] = {
								eGpioKey0,eGpioKey1,
	                          	eGpioKey2,eGpioKey3,
	  	                        eGpioKey4,eGpioKey5,
							    eGpioKey6,eGpioKey7,
							    eGpioKey8,eGpioKey9
							};

static int32_t CLUSTER_List[3] = {
									eClusterKey1 ,eClusterKey2,
									eClusterKey3
								};

/*----------------------------------------------------------------------------*
 * Flash GPIO Interface 									                      *
 *-----------------------------------------------------------------------------*/
#define FLASH_GPIO_BASE      GPIO9
#define FLASH_PCS0_GPIO_PIN  (28U)

#define FLASH_CLK_GPIO	27U
#define FLASH_SDO_GPIO	29U
#define FLASH_SDI_GPIO	30U

#define BYTE_MODE 3

#define 	SECTOR_SIZE 					4096
#define 	SYSTEMINFO_SECTOR_NUMBER0 		0
#define 	SYSTEMINFO_SECTOR_NUMBER1 		1
#define 	SYSTEMINFO_SECTOR_NUMBER2 		2
#define 	SYSTEMINFO_SECTOR_NUMBER3 		3
#define 	SYSTEMINFO_SECTOR_NUMBER4 		4

#define 	SECTOR_ADDRESS0  (SECTOR_SIZE * SYSTEMINFO_SECTOR_NUMBER0)
#define 	SECTOR_ADDRESS1  (SECTOR_SIZE * SYSTEMINFO_SECTOR_NUMBER1)
#define 	SECTOR_ADDRESS2  (SECTOR_SIZE * SYSTEMINFO_SECTOR_NUMBER2)
#define 	SECTOR_ADDRESS3  (SECTOR_SIZE * SYSTEMINFO_SECTOR_NUMBER3)
#define 	SECTOR_ADDRESS4  (SECTOR_SIZE * SYSTEMINFO_SECTOR_NUMBER4)

#ifdef __cplusplus
}
#endif


#endif /* CLUSTER_CONFIG_H_ */
