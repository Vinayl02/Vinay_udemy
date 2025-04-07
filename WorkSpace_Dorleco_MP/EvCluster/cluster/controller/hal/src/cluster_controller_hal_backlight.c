/*-----------------------------------------------------------------------------*
 *	OptM Media Solutions Confidential										   *
 *	Copyright (C) OptM Media Solutions Pvt. Ltd - All Rights Reserved	   	   *
 *																			   *
 *  Dissemination of this information or reproduction or redistribution of 	   *
 *  this material is  strictly forbidden unless prior written permission is	   *
 *  obtained from OptM Media Solutions Pvt Ltd								   *
 *	File Name	:	cluster_controller_hal_backlight.c 								  	   		   
 *  version		: 											              	   *
 *  Date		:	13-Mar-2023													   
 *  Description :  				 										 	   * 
 *                               						                       *
 *																			   *
 *-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*
 * Include Headers				           	                             	   *
 *-----------------------------------------------------------------------------*/
#include "fsl_pwm.h"
#include "fsl_xbara.h"
#include "fsl_gpio.h"
#include "cluster_controller_hal_backlight.h"
#include "cluster_controller_hal_config.h"
#include "fsl_debug_console.h"
/*----------------------------------------------------------------------------* 
 * Macro ,Structure and ENUM definition                   				      *
 *----------------------------------------------------------------------------*/
typedef struct
{
	int32_t 			iIsInitialised;
	uint32_t 			iCurLevel;
	uint32_t 			iMaxLevel;	
	uint8_t* 			pLevelMap;
	PWM_Type 			*pBase;
	uint32_t 			iPwmFreq;
	uint8_t 			iPwmId;
	uint16_t			iPwmOutputSignal_0;
	uint16_t			iPwmOutputSignal_1;
}sBacklight_t;

#define 	BACKLIGHT_PWM_SRC_CLK_FREQ       	CLOCK_GetRootClockFreq(kCLOCK_Root_Bus)
#define 	BACKLIGHT_PWM_CLOCK_DEVIDER 		kPWM_Prescale_Divide_4
#if(HW_REV_VERSION == 1)
#define 	BACKLIGHT_PWM_SUBMODULE 			kPWM_Module_1
#define 	BACKLIGHT_PWM_MODULE_CONTROL 		kPWM_Control_Module_1
#else
#define 	BACKLIGHT_PWM_SUBMODULE 			kPWM_Module_2
#define 	BACKLIGHT_PWM_MODULE_CONTROL 		kPWM_Control_Module_2
#endif
#define 	BACKLIGHT_PWM_CHANNELS 				kPWM_PwmB
#define 	BACKLIGHT_PWM_MODE 					kPWM_SignedCenterAligned
#define 	BACKLIGHT_PWM_NO_OF_CHANNELS 		1

/*----------------------------------------------------------------------------* 
 * Static and global variable definition	                         		  * 
 *----------------------------------------------------------------------------*/
static sBacklight_t   hBl= {0};

/*----------------------------------------------------------------------------* 
 * Local function definition	                         				      * 
 *----------------------------------------------------------------------------*/
static int32_t Backlight_Pwm_Init(sBacklight_t * pHan);
static int32_t Backlight_SetBrightness(sBacklight_t * pHan ,uint8_t iVal);
/*-----------------------------------------------------------------------------*
 * Function	    :											        		   *
 * Params	    :					        		                           *
 * Return value	:									                           *
 * Description	:							                                   *
 *-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Backlight_Init(void)
{
	int32_t iRval = BCU_OK;
	if(hBl.iIsInitialised == 0)
	{
		memset(&hBl, 0 ,sizeof(sBacklight_t));
		Cluster_Controller_Hal_Config_GetDefaultBrightnessLevel(&(hBl.iCurLevel));
		Cluster_Controller_Hal_Config_GetMaxBrightnessLevel(&(hBl.iMaxLevel));		
		if(hBl.iCurLevel > hBl.iMaxLevel)
		{
			hBl.iCurLevel = hBl.iMaxLevel/2;
		}

		hBl.pLevelMap = Cluster_Controller_Hal_Config_GetBrightnessMap();
		iRval = Cluster_Controller_Hal_Config_BacklightGetInterface(&(hBl.iPwmId), &(hBl.iPwmFreq));
		if (iRval == BCU_OK)
		{		
			iRval = Backlight_Pwm_Init(&hBl);
			if (iRval == BCU_OK)
			{
				hBl.iIsInitialised = 1;

			}
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
int32_t Cluster_Controller_Hal_Backlight_DeInit(void)
{
	if(hBl.iIsInitialised == 1)
	{
		Cluster_Controller_Hal_Backlight_Enable(0);
		PWM_Deinit(hBl.pBase,BACKLIGHT_PWM_SUBMODULE);
	}
	memset(&hBl, 0 ,sizeof(sBacklight_t));
	return BCU_OK;
}

/*-----------------------------------------------------------------------------*
* Function	    :											        		   *
* Params	    :					        		                           *
* Return value	:									                           *
* Description	:							                                   *
*-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Backlight_SetBrightnessLevel (uint32_t iValue)
{
	int32_t iRval = BCU_NOK;

	if((hBl.iIsInitialised) && (iValue <= hBl.iMaxLevel))
	{
		iRval = Backlight_SetBrightness(&hBl,hBl.pLevelMap[iValue]);
	}
	return iRval;
}
/*-----------------------------------------------------------------------------*
* Function	    :											        		   *
* Params	    :					        		                           *
* Return value	:									                           *
* Description	:							                                   *
*-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Backlight_SetBrightness(uint8_t iValue)
{
	int32_t iRval = BCU_NOK;

	if((hBl.iIsInitialised) && ((iValue > 0) && (iValue < 100)))
	{
		iRval= Backlight_SetBrightness(&hBl,iValue);
	}
	return iRval;
}

/*-----------------------------------------------------------------------------*
* Function	    :											        		   *
* Params	    :					        		                           *
* Return value	:									                           *
* Description	:							                                   *
*-----------------------------------------------------------------------------*/
int32_t Cluster_Controller_Hal_Backlight_Enable(uint8_t iEnable)
{
	int32_t iRval = BCU_OK;
	if(hBl.iIsInitialised == 1)
	{
		uint8_t iVal =0;
		if(iEnable)
		{
			iVal = hBl.pLevelMap[hBl.iCurLevel];
		}

		iRval = Backlight_SetBrightness(&hBl,iVal);
	}
	else
	{
		iRval = BCU_NOK;
	}
	return iRval;
}

/*-----------------------------------------------------------------------------*
* Function	    :											         		   *
* Params	    :					        		                           *
* Return value	:									                           *
* Description	:							                                   *
*-----------------------------------------------------------------------------*/
static int32_t Backlight_Pwm_Init(sBacklight_t * pHan)
{
	int32_t iRval = BCU_OK;
	uint16_t deadTimeVal =0;

	pwm_config_t 			BlPwmConfig;
	pwm_fault_param_t   	BlFaultConfig;
	pwm_signal_param_t 		BlPwmSignal;

	switch(pHan->iPwmId)
	{
		case 2:
		{
			pHan->pBase= PWM2;
			pHan->iPwmOutputSignal_0 = kXBARA1_OutputFlexpwm2Fault0;
			pHan->iPwmOutputSignal_1 = kXBARA1_OutputFlexpwm2Fault1;
		}
		break;
		case 3:
		{
			pHan->pBase= PWM3;
			pHan->iPwmOutputSignal_0 = kXBARA1_OutputFlexpwm3Fault0;
			pHan->iPwmOutputSignal_1 = kXBARA1_OutputFlexpwm3Fault1;
		}
		break;
		case 4:
		{
			pHan->pBase= PWM4;
			pHan->iPwmOutputSignal_0 = kXBARA1_OutputFlexpwm4Fault0;
			pHan->iPwmOutputSignal_1 = kXBARA1_OutputFlexpwm4Fault1;
		}
		break;
		case 1:		
		default:
		{
			pHan->pBase= PWM1;
			pHan->iPwmOutputSignal_0 = kXBARA1_OutputFlexpwm1Fault0;
			pHan->iPwmOutputSignal_1 = kXBARA1_OutputFlexpwm1Fault1;
		}
		break;
	}

	XBARA_Init(XBARA1);
	XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputLogicHigh, pHan->iPwmOutputSignal_0);
	XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputLogicHigh, pHan->iPwmOutputSignal_1);


	PWM_GetDefaultConfig(&BlPwmConfig);

	BlPwmConfig.prescale 			= BACKLIGHT_PWM_CLOCK_DEVIDER;
	BlPwmConfig.reloadLogic 		= kPWM_ReloadPwmFullCycle;
	BlPwmConfig.pairOperation  		= kPWM_Independent ;/*kPWM_ComplementaryPwmA;*/
	BlPwmConfig.enableDebugMode 	= true;

	/* Initialize submodule 2 */

	iRval = PWM_Init(pHan->pBase, BACKLIGHT_PWM_SUBMODULE, &BlPwmConfig);
	if (iRval == kStatus_Success)
	{

		PWM_FaultDefaultConfig(&BlFaultConfig);

		/* Sets up the PWM fault protection */
		PWM_SetupFaults(pHan->pBase, kPWM_Fault_0, &BlFaultConfig);
		PWM_SetupFaults(pHan->pBase, kPWM_Fault_1, &BlFaultConfig);


		PWM_SetupFaultDisableMap(pHan->pBase, 
								BACKLIGHT_PWM_SUBMODULE, 
								BACKLIGHT_PWM_CHANNELS,
								kPWM_faultchannel_0,
								kPWM_FaultDisable_0 | kPWM_FaultDisable_1 );

		deadTimeVal = (uint16_t)(((uint64_t)BACKLIGHT_PWM_SRC_CLK_FREQ * 650) / 1000000000);
		
		

		BlPwmSignal.pwmChannel        = BACKLIGHT_PWM_CHANNELS;
		BlPwmSignal.level             = kPWM_HighTrue;
		BlPwmSignal.dutyCyclePercent  = hBl.pLevelMap[hBl.iCurLevel];
		BlPwmSignal.deadtimeValue     = deadTimeVal;
		BlPwmSignal.faultState        = kPWM_PwmFaultState0;
		BlPwmSignal.pwmchannelenable  = true;

		iRval = PWM_SetupPwm(pHan->pBase,
							BACKLIGHT_PWM_SUBMODULE,
							&BlPwmSignal,
							BACKLIGHT_PWM_NO_OF_CHANNELS,
							BACKLIGHT_PWM_MODE,
							pHan->iPwmFreq,
							BACKLIGHT_PWM_SRC_CLK_FREQ);
		
		if (iRval == kStatus_Success)
		{
			PWM_SetPwmLdok(pHan->pBase, BACKLIGHT_PWM_MODULE_CONTROL , true);
			PWM_StartTimer(pHan->pBase, BACKLIGHT_PWM_MODULE_CONTROL );
			iRval = BCU_OK;
		}
	}
	else
	{
		iRval = BCU_NOK;
	}

	return iRval;
}



/*-----------------------------------------------------------------------------*
* Function	    :											         		   *
* Params	    :					        		                           *
* Return value	:									                           *
* Description	:							                                   *
*-----------------------------------------------------------------------------*/
static int32_t Backlight_SetBrightness(sBacklight_t * pHan ,uint8_t iVal)
{
	int32_t iRval = BCU_OK;
	if(iVal > 0)
	{
		PWM_UpdatePwmDutycycle(	pHan->pBase,
								BACKLIGHT_PWM_SUBMODULE,
								BACKLIGHT_PWM_CHANNELS,
								BACKLIGHT_PWM_MODE,
								iVal);

		PWM_SetPwmLdok(pHan->pBase, BACKLIGHT_PWM_MODULE_CONTROL , true);
	}
	else
	{
		iRval = BCU_NOK;
	}
	return iRval;	
}
