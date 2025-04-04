#include "board.h"
#include <qul/application.h>
#include <qul/qul.h>
#include <platforminterface/log.h>
#include <fsl_gpio.h>
#include <FreeRTOS.h>
#include <hmimain.h>
#include <hmihandler.h>
#include <queue.h>
#include "fsl_debug_console.h"
#include "cluster_common.h"
#include "cluster_controller_manager.h"
#include "cluster_controller_platform.h"
/*-----------------------------------------------------------------------------*
 * Namespace 					           	                             	   *
 *-----------------------------------------------------------------------------*/
namespace ControllerPlatform {

/*-----------------------------------------------------------------------------*
 * Macro and variable definitions         	                             	   *
 *-----------------------------------------------------------------------------*/
#define MAX_QUEUE_MSG_COUNT				250
#define MSG_BUFFER_SIZE 				sizeof(sMessage_t)+16
#define MQUAUE_WAIT_MAX				    portMAX_DELAY
#define MAX_MESSAGES					MAX_QUEUE_MSG_COUNT


static int iWriteOffset = 0;
static char sMqBuf[MSG_BUFFER_SIZE] = {0};
static sMessage_t sEventMsg[MAX_MESSAGES] = {0};

static QueueHandle_t hAppQueue =  NULL;
static sController_t hController = {0};


/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
void Start()
{
	sMessage_t* pMsgPtr;

	hAppQueue = xQueueCreate(MAX_QUEUE_MSG_COUNT, MSG_BUFFER_SIZE);
	if(hAppQueue != NULL)
	{
		hController.QueueId = hAppQueue;
		Cluster_Controller_Manager_Init(&hController);
		Cluster_Controller_Manager_Start();
		while (true)
		{
			memset(&sMqBuf[0],0, MSG_BUFFER_SIZE);
			if (pdTRUE == xQueueReceive(hAppQueue,&sMqBuf[0], (TickType_t)(MQUAUE_WAIT_MAX)))
			{
				pMsgPtr = &(sEventMsg[iWriteOffset]);

				memset(pMsgPtr,0, sizeof(sMessage_t));

				memcpy ((char *)pMsgPtr, &(sMqBuf[0]), sizeof(sMessage_t));

				if((pMsgPtr->iEventId > 0) && (pMsgPtr->iEventId <= eEvent_Max))
				{

					UI::sendToUI((int)(pMsgPtr->iEventId),
									 (void*)&(pMsgPtr->sData[0]));


					iWriteOffset = (iWriteOffset +1) % MAX_MESSAGES;
				}
				else
				{
					PRINTF("\r\nError: ClusterController: Invalid Event EventID = %d\r\n",pMsgPtr->iEventId);
				}
			}
		}
	}
	else
	{
		PRINTF("\r\nError: ClusterController: App queue creation failed\r\n");
	}
}
} // namespace ClusterController

