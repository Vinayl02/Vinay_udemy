#ifndef CLUSTER_CONTROLLER_HAL_CONFIG_H_
#define CLUSTER_CONTROLLER_HAL_CONFIG_H_


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
 *	File Name	:	cluster_hal_uart.h
 *  version		: 											              	   *
 *  Date		:	03-Mar-2023
 *  Description :  				 										 	   *
 *                               						                       *
 *																			   *
 *-----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*
 * Include Headers				           	                             	  *
 *----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*
 * Structure and enum definations				                              *
 *----------------------------------------------------------------------------*/

 /*----------------------------------------------------------------------------*
  * Macro ,Structure and enum definition                   				      *
  *----------------------------------------------------------------------------*/
#define FLASH_SECTOR_SIZE  4096
#define FLASH_SECTOR_NUMBER  0

 /*----------------------------------------------------------------------------*
 * Methods                      				                              *
 *----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_Config_Init(void);
 

/*----------------------------------------------------------------------------*
 * Function	    :										        	   	      *
 * Params	    :				        		                          	  *
 * Return value	:									                          *
 * Description	:									                          *
 *----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_Config_DeInit(void);
 
 /*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/

int32_t Cluster_Controller_Hal_Config_BtGetInterface(uint32_t* pId, uint32_t* pBaudRate);

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/

int32_t Cluster_Controller_Hal_Config_CanGetInterface(int32_t* pId, uint32_t* pBaudRate);

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
uint32_t Cluster_Controller_Hal_Config_GetMaxCanId(void);

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
sCanConfig_t* Cluster_Controller_Hal_Config_GetCanIdList(void);
/*-----------------------------------------------------------------------------*
* Function	    :											        		   *
* Params	    :					        		                           *
* Return value	:									                           *
* Description	:							                                   *
*-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Config_RtcGetInterface(uint32_t * pId, uint32_t * pBaudRate);
/*-----------------------------------------------------------------------------*
* Function	    :											        		   *
* Params	    :					        		                           *
* Return value	:									                           *
* Description	:							                                   *
*-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Config_RtcGetInterruptConf(int32_t* pVal);


/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
uint32_t* Cluster_Controller_Hal_Config_SwitchGetList(int id);
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Config_BacklightGetInterface(uint8_t* pId, uint32_t* pFreq);

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Config_GetDefaultBrightnessLevel(uint32_t* pVal);

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Config_GetMaxBrightnessLevel(uint32_t* pVal);

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
uint8_t*   Cluster_Controller_Hal_Config_GetBrightnessMap(void);

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
void Cluster_Controller_Hal_Config_GetStackSize(uint16_t* pVal);
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
uint32_t Cluster_Controller_Hal_Config_FlashInterface(uint32_t * pBase,
													  uint32_t * pPCS0,
													  uint32_t * pClk,
													  uint32_t * pSdo,
													  uint32_t * pSdi);

/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
uint32_t Cluster_Controller_Hal_Config_GetFlashAddress(uint32_t iSectorAdd );

/*-----------------------------------------------------------------------------*
  Function	    :											        		   
  Params	    :					        		                           
  Return value	:									                           
  Description	:							                                   
 *-----------------------------------------------------------------------------*/
 int32_t Cluster_Controller_Hal_Config_GetVersion(char* pBuf,int iSize);
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Config_GetSwHwVersion(uint16_t * iSwVersion,uint16_t * iHwVersion);

#ifdef __cplusplus
}
#endif


#endif /* CLUSTER_CONTROLLER_HAL_CONFIG_H_ */
