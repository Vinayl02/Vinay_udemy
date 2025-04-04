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

#include "cluster_controller_hal_romapi.h"
#include "fsl_romapi.h"
#include "fsl_debug_console.h"
#include "fsl_cache.h"

/*----------------------------------------------------------------------------* 
 * Static and global variable definition	                         		  * 
 *----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
void Cluster_Controller_Hal_Romapi_FirmwareDownload_Init(void)
{
	PRINTF("\r\n");
	PRINTF("\r\n***************************************\n");
	PRINTF("\r\n*                                     *\n");
	PRINTF("\r\n*  ENTERING SERIAL DOWNLOADER MODE    *\n");
	PRINTF("\r\n*                                     *\n");
	PRINTF("\r\n***************************************\n");
	PRINTF("\n");


    ROM_API_Init();
    uint32_t arg = 0xeb100000;//Enter Serial down loader mode and select UART as the communication peripheral:
    ROM_RunBootloader(&arg);

}


