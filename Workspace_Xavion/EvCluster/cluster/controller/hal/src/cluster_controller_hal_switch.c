/*-----------------------------------------------------------------------------*
 *	OptM Media Solutions Confidential										   *
 *	Copyright (C) OptM Media Solutions Pvt Ltd - All Rights Reserved.	   	   *
 *  Dissemination of this information or reproduction or redistribution of 	   *
 *  this material is  strictly forbidden unless prior written permission is	   *
 *  obtained from OptM Media Solutions Pvt Ltd								   *
 *	File Name	:	cluster_controller_hal_bt.c 							   *
 *  version		: 											              	   *
 *  Date		:	03-Mar-2023												   *
 *  Description :  				 										 	   * 
 *                               						                       *
 *-----------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------*
 * Include Headers				           	                             	   *
 *-----------------------------------------------------------------------------*/
/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/* Freescale includes. */
#include "MIMXRT1176_cm7.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_gpio.h"
#include "fsl_gpt.h"

#include "cluster_common.h"
#include "cluster_controller_hal_switch.h"
#include "cluster_controller_hal_config.h"
/*----------------------------------------------------------------------------* 
 * Macro ,Structure and Enum definition                   				      *
 *----------------------------------------------------------------------------*/
#define EXN_SW_GPIO                  GPIO13
#define EXN_SW_IRQ                   GPIO13_Combined_0_31_IRQn
#define EXN_SW_IRQHandler            GPIO13_Combined_0_31_IRQHandler

#define NAV_SW_GPIO                  GPIO4
#define NAV_SW_IRQ                   GPIO4_Combined_0_15_IRQn
#define NAV_SW_IRQHandler            GPIO4_Combined_0_15_IRQHandler

#define EXID_MAX                       10
#define NAVID_MAX                      3 //5

#define SWITCH_MAX_QUEUE_MSG_COUNT      10
#define SWITCH_MSG_QUEUE_BUFFER_SIZE    4

#define ANALOG_INPUT 0

typedef struct
{
	bool				bRefresh;
	GPIO_Type           *base;
	uint32_t * 	        pHwInput;
	uint32_t *          pNavInput;
	int32_t 			iIsInitialised;
	QueueHandle_t       qRxMsg;
	QueueHandle_t       qSwMsg;
	TaskHandle_t 		hThread;
}HalSwitch_t;

/*----------------------------------------------------------------------------* 
 * Static and global variable definition	                         		  * 
 *----------------------------------------------------------------------------*/
static HalSwitch_t   hSwitch= {0};
char sExtBuf[SWITCH_MSG_QUEUE_BUFFER_SIZE] = {0};
char sNavBuf[SWITCH_MSG_QUEUE_BUFFER_SIZE] = {0};
static  sHwInputs_t sStatus ={0};
static  sNavSwitch_t  sNavInput ={0};
/*----------------------------------------------------------------------------* 
 * Local function definition	                         				      * 
 *----------------------------------------------------------------------------*/
static void Switch_ReceiveThread(void *arg);
static int32_t Switch_InitIo(HalSwitch_t *pHan);
static int32_t Switch_EnableIo(HalSwitch_t *pHan);
static int32_t Switch_Configure(HalSwitch_t *pHan);
static void Switch_ExternalInput(HalSwitch_t *pHan);
static void Switch_NavigationFunc(HalSwitch_t *pHan);
void EXN_SW_IRQHandler(void);
void NAV_SW_IRQHandler(void);
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Switch_Init(void)
{
	int32_t iRval = BCU_OK;
	if(hSwitch.iIsInitialised == 0)
	{
		memset(&hSwitch, 0 ,sizeof(HalSwitch_t));
		iRval = Switch_Configure(&hSwitch);
		if(iRval == BCU_OK )
		{
			hSwitch.iIsInitialised = 1;
		}
	}
	return iRval;
}
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Switch_DeInit(void)
{
	if(hSwitch.iIsInitialised > 0)
	{
		if( &hSwitch.hThread != NULL )
		{
			vTaskDelete( hSwitch.hThread );
		}
		if(hSwitch.qSwMsg != NULL)
		{
			vQueueDelete(hSwitch.qSwMsg);
		}
	}
	memset(&hSwitch, 0, sizeof(HalSwitch_t));
	return BCU_OK;
}
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Switch_SetReceiveQueueHandle(QueueHandle_t  hSwitchQueue)
{
	int32_t iRval = BCU_OK;
	if (hSwitch.iIsInitialised > 0)
	{
		hSwitch.qSwMsg	= hSwitchQueue;
		iRval = BCU_OK;
	}
	return iRval;
}
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
void EXN_SW_IRQHandler(void)
{
	BaseType_t bTaskWoken = pdFALSE;

	sExtBuf[0] = 2;

	if(hSwitch.qRxMsg != NULL)
	{
		bool rval = xQueueSendFromISR(hSwitch.qRxMsg, (char*)&sExtBuf, &bTaskWoken);
		if( rval != pdPASS )
		{
			PRINTF("\n\r EXN_SW : Queue overflow\n\r");
		}
	}

//	GPIO_PortClearInterruptFlags(EXN_SW_GPIO,(1U << 0U) | (1U << 3U) | (1U << 4U) | (1U << 5U) | (1U << 6U) |
//			                                  (1U << 7U) |(1U << 8U) | (1U << 9U) |(1U << 10U) |(1U << 11U));
#if(HW_REV_VERSION == 1)
	GPIO_PortClearInterruptFlags(EXN_SW_GPIO,0xFF9);
#else
	GPIO_PortClearInterruptFlags(EXN_SW_GPIO,0x1FF9);
#endif

}
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
void NAV_SW_IRQHandler(void)
{
	BaseType_t bTaskWoken = pdFALSE;

	sNavBuf[0] = 1;

	if(hSwitch.qRxMsg != NULL)
	{
		bool rval = xQueueSendFromISR(hSwitch.qRxMsg, (char*)&sNavBuf, &bTaskWoken);
		if( rval != pdPASS )
		{
			PRINTF("\n\r NAV_SW : Queue overflow\n\r");
		}
	}
    //TBD -> consider only 3 inputs

	//GPIO_PortClearInterruptFlags(NAV_SW_GPIO, (1U << 3U) | (1U << 4U) | (1U << 5U) | (1U << 6U) | (1U << 7U));
	GPIO_PortClearInterruptFlags(NAV_SW_GPIO,0x38); //0xF8
}
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
static void Switch_ReceiveThread(void *arg)
{
	HalSwitch_t*   	pHan 		= (HalSwitch_t*)(arg);
	uint8_t iCount = 0;
	char sRxBuf[SWITCH_MSG_QUEUE_BUFFER_SIZE] = {0};
	TickType_t maxDelay = pdMS_TO_TICKS(2000);

	xQueueReceive(pHan->qRxMsg, (char*)&sRxBuf[0], pdMS_TO_TICKS(3000));
	Switch_EnableIo(pHan);

	while (1)
	{
		memset(sRxBuf, 0,SWITCH_MSG_QUEUE_BUFFER_SIZE);
		if (xQueueReceive(pHan->qRxMsg, (char*)&sRxBuf[0], maxDelay) == pdTRUE)
		{
			if(sRxBuf[0] == 1)
			{
				Switch_NavigationFunc(pHan);
			}
			else if(sRxBuf[0] == 2)
			{
				pHan->bRefresh = 0;
				Switch_ExternalInput(pHan);
			}
		}
		else
		{
			pHan->bRefresh = 1;
			Switch_ExternalInput(pHan);
			Switch_NavigationFunc(pHan);
			if(iCount >= 4)
			{
				maxDelay = pdMS_TO_TICKS(5000);
			}
			else
			{
				iCount++;
			}
		}
	}
}
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
static void Switch_ExternalInput(HalSwitch_t *pHan)
{
	sMessage_t sMsg = {0};
	memset(&sStatus,0,sizeof(sHwInputs_t));

	sStatus.bRefresh =  pHan->bRefresh;

	if(GPIO_PinRead(EXN_SW_GPIO, pHan->pHwInput[0]) == 0)
	{
		sStatus.bInput1 = 1;
	}

	if(GPIO_PinRead(EXN_SW_GPIO, pHan->pHwInput[1]) == 0)
	{
		sStatus.bInput2 = 1;
	}

	if(GPIO_PinRead(EXN_SW_GPIO, pHan->pHwInput[2]) == 0)
	{
		sStatus.bInput3 = 1;
	}

	if(GPIO_PinRead(EXN_SW_GPIO, pHan->pHwInput[3]) == 0)
	{
		sStatus.bInput4 = 1;
	}

	if(GPIO_PinRead(EXN_SW_GPIO, pHan->pHwInput[4]) == 0)
	{
		sStatus.bInput5 = 1;
	}

	if(GPIO_PinRead(EXN_SW_GPIO, pHan->pHwInput[5]) == 0)
	{
		sStatus.bInput6 = 1;
	}

	if(GPIO_PinRead(EXN_SW_GPIO, pHan->pHwInput[6]) == 0)
	{
		sStatus.bInput7 = 1;
	}

	if(GPIO_PinRead(EXN_SW_GPIO, pHan->pHwInput[7]) == 0)
	{
		sStatus.bInput8 = 1;
	}

	if(GPIO_PinRead(EXN_SW_GPIO, pHan->pHwInput[8]) == 0)
	{
		sStatus.bInput9 = 1;
	}

	if(GPIO_PinRead(EXN_SW_GPIO, pHan->pHwInput[9]) == 0)
	{
		sStatus.bInput10 = 1;
	}

	if(hSwitch.qSwMsg != NULL)
	{
		sMsg.iEventId 			= EXTERNAL_SW;

		memcpy((char *)sMsg.sData,(char*)(&sStatus),sizeof(sHwInputs_t));

		xQueueSend(pHan->qSwMsg, &sMsg, (TickType_t)(5000));
	}
}
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
static void Switch_NavigationFunc(HalSwitch_t *pHan)
{
	sMessage_t sMsg = {0};

	if(GPIO_PinRead(NAV_SW_GPIO, pHan->pNavInput[0]) == 0)
	{
		sNavInput.iVal |= eKey_Right ;
	}
	else
	{
		sNavInput.iVal &= (uint8_t)(~eKey_Right);
	}

	if(GPIO_PinRead(NAV_SW_GPIO, pHan->pNavInput[1]) == 0)
	{
		sNavInput.iVal |= eKey_Ok ;
	}
	else
	{
		sNavInput.iVal &= (uint8_t)(~eKey_Ok) ;
	}

	if(GPIO_PinRead(NAV_SW_GPIO, pHan->pNavInput[2]) == 0)
	{
		sNavInput.iVal |= eKey_Left ;
	}
	else
	{
		sNavInput.iVal &= (uint8_t)(~eKey_Left) ;
	}

	sMsg.iEventId 			= NAVIGATION_SW;
	memcpy((char *)sMsg.sData,(char*)(&sNavInput),sizeof(sNavSwitch_t));

   if(hSwitch.qSwMsg != NULL)
   {
	   xQueueSend(pHan->qSwMsg, &sMsg, (TickType_t)(5000));
   }
}
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
static int32_t Switch_Configure(HalSwitch_t *pHan)
{
	int32_t iRet = 0;
	int32_t iRval = BCU_OK;
	uint16_t iStackSize = 0;

	iRval = Switch_InitIo(pHan);
	pHan->qRxMsg = xQueueCreate(SWITCH_MAX_QUEUE_MSG_COUNT,SWITCH_MSG_QUEUE_BUFFER_SIZE);
	Cluster_Controller_Hal_Config_GetStackSize(&iStackSize);

	iRet= xTaskCreate(Switch_ReceiveThread,
						"switch",
						iStackSize,
						&hSwitch,
						configMAX_PRIORITIES - 2,
						&hSwitch.hThread);
	if(iRet == 1 )
	{
		iRval = BCU_OK;
		//Switch_EnableIo(pHan);
	}
	else
	{
		iRval =iRet;
	}
	return iRval;
}

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
static int32_t Switch_InitIo(HalSwitch_t *pHan)
{
	uint8_t i =0;

	gpio_pin_config_t sConfig = {
		        kGPIO_DigitalInput,
		        0,
				kGPIO_IntRisingOrFallingEdge,
		    };

	pHan->pHwInput    =  Cluster_Controller_Hal_Config_SwitchGetList(0); //Externalinput

	if(pHan->pHwInput != NULL)
	{
		for( i = 0 ; i < EXID_MAX ; i++)
		{
			GPIO_PinInit(EXN_SW_GPIO,pHan->pHwInput[i], &sConfig);
		}

		NVIC_SetPriority(EXN_SW_IRQ, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1);
	}

#if CLUSTER_NAVIGATION_SWITCH_ENABLE
	pHan->pNavInput =  Cluster_Controller_Hal_Config_SwitchGetList(1); //Navigation
	
	if(pHan->pNavInput != NULL)
	{
		for( i = 0 ; i < NAVID_MAX ; i++)
		{
			GPIO_PinInit(NAV_SW_GPIO,pHan->pNavInput[i], &sConfig);
		}
		NVIC_SetPriority(NAV_SW_IRQ, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1);
	}
#endif
	return BCU_OK;
}

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
static int32_t Switch_EnableIo(HalSwitch_t *pHan)
{
	int i =0;
	EnableIRQ(EXN_SW_IRQ);

	for(i = 0 ; i < EXID_MAX; i++)
	{
		GPIO_PortEnableInterrupts(EXN_SW_GPIO, 1U << pHan->pHwInput[i]);
	}

#if CLUSTER_NAVIGATION_SWITCH_ENABLE
	EnableIRQ(NAV_SW_IRQ);
	for(i = 0 ; i < NAVID_MAX; i++)
	{
		GPIO_PortEnableInterrupts(NAV_SW_GPIO, 1U << pHan->pNavInput[i]);
	}
#endif
	return BCU_OK;
}
