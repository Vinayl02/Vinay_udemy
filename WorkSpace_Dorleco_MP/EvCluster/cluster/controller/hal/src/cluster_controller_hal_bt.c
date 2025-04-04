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
#include "cluster_common.h"
#include <fsl_lpuart_freertos.h>
#include "MIMXRT1176_cm7.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "clock_config.h"
#include "fsl_lpuart.h"
#include "cluster_controller_hal_bt.h"
#include "cluster_controller_hal_config.h"



/*----------------------------------------------------------------------------*
 * Macro ,Structure and enum definition                   				      *
 *----------------------------------------------------------------------------*/
#define BT_task_PRIORITY        (configMAX_PRIORITIES - 2)
#define BT_RECEIVE_BUF_SIZE 	256

typedef struct
{
	int32_t 		      iIsInitialised;
	int32_t				  iIsUartConfigured;
	uint8_t 		      sBuf[32];
	QueueHandle_t         qRxMsg;
	TaskHandle_t 		  hThread;
	lpuart_rtos_handle_t  hUart;
	struct _lpuart_handle hUartTemp;
} hal_bt_t;

static hal_bt_t   hBt = {0};

/*----------------------------------------------------------------------------* 
 * Static and global variable definition	                         		  * 
 *----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------* 
 * Local function definition	                         				      * 
 *----------------------------------------------------------------------------*/
static int32_t BT_Configure_Uart(hal_bt_t * pHan);
static void BT_ReceiveThread(void *arg);
/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Bt_Init							   *
 * Params	    :	void				        		                       *
 * Return value	:	On success return BCU_OK else return BCU_NOK			   *
 * Description	:	In this method, Initialization and configuration of uart
 * 					will take place for blue tooth low energy communication.   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Bt_Init(void)
{
    int32_t iRval = BCU_NOK;
    if(hBt.iIsInitialised == 0)
    {
		memset(&hBt, 0 ,sizeof(hal_bt_t));
		iRval = BT_Configure_Uart(&hBt);
		if(iRval == BCU_OK )
		{
			hBt.iIsInitialised = 1;
			hBt.iIsUartConfigured =1;
		}
		else
		{
			PRINTF("ERROR: BT_Configure_Uart\r\n");
		}
	}
    else
    {
    	PRINTF("ERROR: Cluster_Hal_Bt_Init iIsInitialised : %d\r\n", hBt.iIsInitialised);
    }
    return iRval;
}
/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Bt_DeInit		        		   *
 * Params	    :	void				        		                       *
 * Return value	:	On success return BCU_OK else return BCU_NOK			   *
 * Description	:	In this method, Deinitialization of uart and cancellation
 *					will take place for blue tooth low energy communication.   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Bt_DeInit(void)
{
	if(hBt.iIsInitialised > 0)
	{
		if( &hBt.hThread != NULL )
		{
		    vTaskDelete( hBt.hThread );
		}
	}
	memset(&hBt, 0, sizeof(hal_bt_t));
    return BCU_OK;
}
/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Bt_SetReceiveQueueHandle			   *
 * Params	    :	QueueHandle_t  hBtQueue				        		       *
 * Return value	:	On success return BCU_OK else return BCU_NOK			   *
 * Description	:	In this method, Creating of thread to get the data from
 * 					uart, and accessing of queue id to send data to ble proc.  *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Bt_SetReceiveQueueHandle(QueueHandle_t  hBtQueue)
{
	int32_t iRval = BCU_NOK;
	if (hBt.iIsInitialised > 0)
	{
		if(hBtQueue!= NULL)
		{
			hBt.qRxMsg	= hBtQueue;
			int32_t iRet = 0;
			uint16_t iStackSize = 0;

			Cluster_Controller_Hal_Config_GetStackSize(&iStackSize);

			iRet = xTaskCreate(BT_ReceiveThread,
									"BT_ReceiveThread", 
									iStackSize,
									&hBt, 
									BT_task_PRIORITY, 
									&hBt.hThread);
			if(iRet == 1)
			{
				iRval = BCU_OK;
			}
			else
			{
				iRval = iRet;
			}
		}
	}
	return iRval;
}
/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Bt_Enable		        		   *
 * Params	    :	int32_t iEnable				        		               *
 * Return value	:	On success return BCU_OK else return BCU_NOK		       *
 * Description	:	In this method, Enable and disable of BLE module will take
 * 					place.						                               *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Bt_Enable(int32_t iEnable)
{
	int32_t iRval = BCU_OK;
	(void)iEnable;
	return iRval;
}
/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Bt_SendMessage					   *
 * Params	    :	int8_t *pMsg, int32_t iLen				        		   *
 * Return value	:	On success return BCU_OK else return BCU_NOK			   *
 * Description	:	In this method, The sending of data to Ble from MCU will
 * 					take place through using of FreeRtos API.				   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Bt_SendMessage(int8_t *pMsg, uint32_t iLen)
{
	int32_t iRval = BCU_NOK;

	if((hBt.iIsUartConfigured != 0) && (pMsg != NULL ) && (iLen > 0))
	{
		iRval = LPUART_RTOS_Send(&(hBt.hUart), (uint8_t *)pMsg, iLen);
	}
	return iRval;
}

/*-----------------------------------------------------------------------------*
 * Function	    :	BT_ReceiveThread										   *
 * Params	    :	void *arg				        		                   *
 * Return value	:	void								                       *
 * Description	:	In this method, The receiving data from BLE will take place
 * 					in polling mode, and sending of received data into Ble_proc
 * 					to process the received data.						       *
 *-----------------------------------------------------------------------------*/
static void BT_ReceiveThread(void *arg)
{
    hal_bt_t*  pHan = (hal_bt_t*)(arg);
	
	unsigned char sBuf[BT_RECEIVE_BUF_SIZE];
	
	size_t numBytes = 0;
	
	memset(&sBuf[0],0,BT_RECEIVE_BUF_SIZE);
	
	LPUART_RTOS_SetRxTimeout(&(pHan->hUart), 500, 1);
	
	int32_t iRetval =0;

	while(1)
	{
		LPUART_RTOS_Receive(&(pHan->hUart),
							 sBuf,
							 BT_RECEIVE_BUF_SIZE,
							 &numBytes);
		if (numBytes > 0)
		{
			PRINTF("\r\nReceived Data : %s\r\n",sBuf);
			if(pHan->qRxMsg != NULL)
			{
				iRetval = xQueueSend(pHan->qRxMsg,sBuf,100);
				if(iRetval != 1)
				{
					PRINTF("ERROR: xQueueSend\r\n");
				}
			}
			memset(&sBuf[0],0,BT_RECEIVE_BUF_SIZE);
		}


	}
	LPUART_RTOS_Deinit(&(hBt.hUart));
	vTaskSuspend(hBt.hThread);
}

/*-----------------------------------------------------------------------------*
 * Function	    :	BT_Configure_Uart							     		   *
 * Params	    :	hal_bt_t * pHan			        		                   *
 * Return value	:	On success return BCU_OK else return BCU_NOK			   *
 * Description	:	In this method, configuration of uart and initialization
 * 					of LPUART_RTOS_Init Api will take place. 				   *
 *-----------------------------------------------------------------------------*/
static int32_t BT_Configure_Uart(hal_bt_t * pHan)
{
	int32_t iRetVal = BCU_OK;
	uint32_t iId;
	uint32_t iUart;
	lpuart_rtos_config_t sUart = {0};
	
	iRetVal = Cluster_Controller_Hal_Config_BtGetInterface(&iId,&iUart);
	if(iRetVal == BCU_OK)
	{
		switch(iId)
		{
			case 1:
			{
				sUart.base = LPUART1;
				sUart.srcclk = CLOCK_GetRootClockFreq(kCLOCK_Root_Lpuart1);
				NVIC_SetPriority(LPUART1_IRQn, 5);
			}
			break;
			case 2:
			{
				sUart.base = LPUART2;
				sUart.srcclk = CLOCK_GetRootClockFreq(kCLOCK_Root_Lpuart2);
				NVIC_SetPriority(LPUART2_IRQn, 5);
			}
			break;
			case 3:
			{
				sUart.base = LPUART3;
				sUart.srcclk = CLOCK_GetRootClockFreq(kCLOCK_Root_Lpuart3);
				NVIC_SetPriority(LPUART3_IRQn, 5);
			}
			break;
			case 10:
			{
				sUart.base = LPUART10;
				sUart.srcclk = CLOCK_GetRootClockFreq(kCLOCK_Root_Lpuart10);
				NVIC_SetPriority(LPUART10_IRQn, 5);
			}
			break;
			default:
			{
				sUart.base = LPUART10;
				sUart.srcclk = CLOCK_GetRootClockFreq(kCLOCK_Root_Lpuart10);
				NVIC_SetPriority(LPUART10_IRQn, 5);
			}
			break;
		}

		sUart.baudrate                 = iUart;
		sUart.parity                   = kLPUART_ParityDisabled;
		sUart.stopbits                 = kLPUART_OneStopBit;
		sUart.buffer                   = pHan->sBuf;
		sUart.buffer_size              = sizeof(pHan->sBuf);
		sUart.rx_timeout_constant_ms   = 1;
		sUart.rx_timeout_multiplier_ms = 1;
		sUart.tx_timeout_constant_ms   = 20;
		sUart.tx_timeout_multiplier_ms = 1;
		sUart.enableRxRTS              = true;
		sUart.enableTxCTS              = true;
		sUart.txCtsSource         	   = kLPUART_CtsSourcePin;
		sUart.txCtsConfig              = kLPUART_CtsSampleAtIdle;

		iRetVal = LPUART_RTOS_Init(&(pHan->hUart), &(pHan->hUartTemp), &sUart);

		if(iRetVal != BCU_OK)
		{
			PRINTF("ERROR:LPUART_RTOS_Init iRtval:  %d\r\n", iRetVal);

		}
	}
	else
	{
		PRINTF("Cluster_Hal_Config_Get_BtInterface\r\n");
	}
	return iRetVal;
}

