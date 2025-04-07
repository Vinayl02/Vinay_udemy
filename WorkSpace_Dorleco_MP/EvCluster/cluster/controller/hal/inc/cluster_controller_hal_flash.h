#ifndef CLUSTER_CONTROLLER_HAL_FLASH_H_
#define CLUSTER_CONTROLLER_HAL_FLASH_H_

/*-----------------------------------------------------------------------------*
 *	OptM Media Solutions Confidential										   *
 *	Copyright (C) OptM Media Solutions Pvt. Ltd - All Rights Reserved	   	   *
 *																			   *
 *  Dissemination of this information or reproduction or redistribution of 	   *
 *  this material is  strictly forbidden unless prior written permission is	   *
 *  obtained from OptM Media Solutions Pvt Ltd								   *
 *	File Name	:	cluster_hal_spi.h
 *  version		: 											              	   *
 *  Date		:	03-Mar-2023
 *  Description :  				 										 	   *
 *                               						                       *
 *																			   *
 *-----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*
 * Include Headers				           	                             	  *
 *----------------------------------------------------------------------------*/
#include <stdio.h>

/*----------------------------------------------------------------------------*
 * Structure and enum definations				                              *
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*
 * Methods                      				                              *
 *----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Flash_Init			     		   *
 * Params	    :	void				        		                       *
 * Return value	:	On success return BCU_OK else it will return BCU_NOK       *
 * Description	:	In this method, Initializes secondary flash(NVM) including
 * 					GPIO's to interface the flash.				               *
 *-----------------------------------------------------------------------------*/
int Cluster_Controller_Hal_Flash_Init();
/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Flash_DeInit			     		   *
 * Params	    :	void				        		                       *
 * Return value	:	On success return BCU_OK else it will return BCU_NOK       *
 * Description	:	In this method, Deinitializes secondary flash(NVM).	       *
 *-----------------------------------------------------------------------------*/
int Cluster_Controller_Hal_Flash_DeInit();
/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Flash_ReadProp			     	*
 * Params	   	    :	int iParam,uint8_t * pBuf,int iSize and int iAddr       *
 * Return value	:	On success return BCU_OK else it will return BCU_NOK    *
 * Description	    :	In this method, Reads the manufacturer data asper
 * 					manufacturer specific.					     		    *
 *-----------------------------------------------------------------------------*/
int  Cluster_Controller_Hal_Flash_ReadProp( int iParam,
											uint8_t* pBuf,
											int32_t iSize,
											uint32_t iAddr);

/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Flash_WriteData			        *
 * Params	    	:	uint8_t* pBuf, int iSize and uint32_t iAddr				*
 * Return value	:	On success return BCU_OK else it will return BCU_NOK    *
 * Description		:	In this method, Write the data into secondary flash
 * 					(NVM) with respect to secondary flash(NVM) manufacturer
 * 					specific.				                                *
 *-----------------------------------------------------------------------------*/
int  Cluster_Controller_Hal_Flash_WriteData(uint8_t* pBuf,
										   int iSize,
										   uint32_t iAddr);
/*-----------------------------------------------------------------------------*
 * Function	    :	Cluster_Controller_Hal_Flash_Erase			     		*
 * Params	   	    :	int iAddr				        		                *
 * Return value	:	On success return BCU_OK else it will return BCU_NOK    *
 * Description	    :	In this method, Erases the secondary flash(NVM) before
 * 					writing into the flash, and follows the  secondary flash
 * 					(NVM) asper manufacturer specific.				        *
 *-----------------------------------------------------------------------------*/
int  Cluster_Controller_Hal_Flash_Erase(uint32_t iAddr);
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int  Cluster_Controller_Hal_Flash_ReadData(uint8_t * pBuf,int iSize , uint32_t iAddr);
#endif /* CLUSTER_CONTROLLER_HAL_FLASH_H_ */
