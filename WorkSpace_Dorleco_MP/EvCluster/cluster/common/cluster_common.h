#ifndef CLUSTER_COMMON_H_
#define CLUSTER_COMMON_H_


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
 *	File Name	:	cluster_common.h
 *  version		: 											              	   *
 *  Date		:	03-Mar-2023
 *  Description :  				 										 	   *
 *                               						                       *
 *																			   *
 *-----------------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"
#include <stdbool.h>

#define   	BCU_OK    0
#define   	BCU_NOK   1

#define  EXTERNAL_SW     0
#define  NAVIGATION_SW   1

#define  VEHICLE_DTC_INFO              3
#define  VEHICLE_SEPC_INFO_MEMSEC      2
#define  CLUSTER_INFO_MEMSEC1          1
#define  CLUSTER_INFO_MEMSEC0          0

typedef enum
{
	 eModule_None		= 0,	
	 eModule_Bt			= 1 << 0,
	 eModule_Can		= 1 << 1,
	 eModule_Rtc    	= 1 << 2,
	 eModule_Switch   	= 1 << 3,
	 eModule_BackLight  = 1 << 4,
	 eModule_Flash      = 1 << 5,
	 eModule_Uds        = 1 << 6
}Cluster_Modules_t;

typedef enum
{
    eEvent_None				= 0,
	eEvent_Motor			= 1,
	eEvent_Battery			= 2,
	eEvent_DriveMode		= 3,
	eEvent_DeviceStatus		= 4,
	eEvent_Indicator		= 5,
	eEvent_HwIndicator		= 6,
	eEvent_Message			= 7,
	eEvent_Standby			= 8,
	eEvent_Fault			= 9,
	eEvent_Call				= 10,
	eEvent_TurnByTurnNav	= 11,
	eEvent_NavSwitch		= 12,
	eEvent_Time             = 13,
	eEvent_Alert			= 14,
	eEvent_DeviceInfo       = 15,
	eEvent_BTMessage        = 16,
	eEvent_Charge			= 17,
//customer specific
	eEvent_BMS              = 18,
	eEvent_Indicator1		= 19,
	eEvent_Indicator2		= 20,
	eEvent_Indicator3		= 21,
	eEvent_Max				= 22,
}eClusterEvents_t;

typedef enum
{
	eSecurityAccess			        = 0x27,
	eDiagnosticSessionControl       = 0x10,
	eProgrammingSession             = 0x02,
	eKeyValue			            = 0x1010,
	eRequestSeed		            = 0x01,
	eSendKey		                = 0x02,
	eFlashMode    					= 0x11,
	eRtcSetTime   					= 0x12,
	eFactoryReset 					= 0x13,
	eBacklightSet 					= 0x14,
	eHardwareStatus 				= 0x15,
	eReadSwHwVersion 				= 0x16,
	eWriteDataByIdentifier          = 0x2E,
	eSlNoIdentifier                 = 0x1F,
	eVINIdentifier                  = 0x2F,
	eDateIdentifier				    = 0x3F,
	eReadDataByIdentifier           = 0x22,
	eReadDTCInformation             = 0x19,
	eReportDTCByStatusMask          = 0x02,
	eClearDiagnosticInformation     = 0x14,
}eClusterUdsServices_t;

typedef enum
{
	eSubFunNotSupported		        = 0X12,
	eIncorrectMesLenOrInavlidFormat	= 0x13,
	eConditiosNotCorrect			= 0x22,
	eReqSeqError					= 0x24,
	eReqOutOFRange					= 0x31,
	eInvalidKey						= 0x35,
	eExceededNumOfAttempts			= 0x36,
	eReqTimeDelayNotExp				= 0x37,
}eClusterUdsNRCs_t;

typedef enum
{
	eCanFault 	 	= 1 << 0,
	eRtcFault 		= 1 << 1,
	eSwitchFault    = 1 << 2,
    eBleFault       = 1 << 3,
	eGsmFault       = 1 << 4,
}eClusterDTCInfo_t;

typedef enum
{
	eInput1  = 0x01,
	eInput2  = 0x02,
	eInput3  = 0x03,
	eInput4  = 0x04,
	eInput5  = 0x05,
	eInput6  = 0x06,
	eInput7  = 0x07,
	eInput8  = 0x08,
	eInput9  = 0x09,
	eInput10 = 0x10,
	eInputs_All = 0x11,
}eHwSubid_t;

typedef enum
{
	eDriveMode_None     = 0,
    eDriveMode_Neutral  = 1,
    eDriveMode_Drive    = 2,
	eDriveMode_Reverse  = 3,
	eDriveMode_Sport    = 4,
    eDriveMode_Invalid  = 5,
}eDriveode_t;

typedef enum
{
#if(CLUSTER_KEYIGNITION_SUPPORTED == 1)
	eGpioKey0		= 12U, //evk ignition via GPIO
#else
	eGpioKey0		= 0U, //mapped to wakeup
#endif
	eGpioKey1       = 3U,
	eGpioKey2		= 4U,
	eGpioKey3       = 5U,
	eGpioKey4		= 6U,
	eGpioKey5       = 7U,
	eGpioKey6       = 8U,
	eGpioKey7       = 9U,
	eGpioKey8       = 10U,
	eGpioKey9       = 11U,
}eGPIOKey_t;

typedef enum
{
	eClusterKey1    = 3U,
	eClusterKey2    = 4U,
	eClusterKey3    = 5U,
}eClusterKey_t;

typedef enum
{
	eRtc_Init     = 1,
	eRtc_State    = 2,
	eRtc_Param    = 3,
	eRtc_Save     = 4,
}eRtcSet_t;

typedef enum
{
	eKey_Left     = 1,
	eKey_Ok   	  = 1 << 1,
	eKey_Right    = 1 << 2,
}eSwitch_t;

typedef enum
{
	eSpeedInfo 	 	 =   1,
	eBrightnessInfo  =   2,
	eBLEStatusInfo   =   3,
	eGsmStatusInfo   =   4,
	eClusterVersInfo =   5,
	eResetInfo       =   6,
	eProductInfo     =   7,
	eDTCInfo         =   8,
} eSystemInfo_t;

typedef struct
{
	uint32_t iId;
	uint32_t iMode;
}sCanConfig_t;

typedef struct
{
	bool bRefresh;
	bool bInput1;
	bool bInput2;
	bool bInput3;
	bool bInput4;
	bool bInput5;
	bool bInput6;
	bool bInput7;
	bool bInput8;
	bool bInput9;
	bool bInput10;
	bool bInput11;
}sHwInputs_t;
typedef struct
{
	uint32_t iTextCode;
}sTextInfo_t;

 typedef struct
 {
	 uint8_t iVal;//refer eSwitch_t
 }sNavSwitch_t;

typedef struct
{
    int32_t iCanId;
    uint32_t iDataLen;
    uint8_t iFormat; //0->Std, 1 ->Extended
    uint8_t sData[8];
}sCanMessage_t;

typedef struct
{
    uint32_t iSpeed;
    uint32_t iOdo;
    uint32_t iTrip;
    uint32_t iRange;
    uint32_t iRpm;
    uint32_t iPower;
    uint32_t iTemp;
}sMotorInfo_t;

typedef struct
{
	int32_t	iDriveVal;
	int32_t iGearVal;
}sDriveMode_t;

typedef struct
{
	bool bType;
	bool bGunlock;
	bool bStatus;
	uint32_t iTime;
	uint32_t iEnergyCon;
}sChargeInfo_t;

typedef struct
{
    uint32_t iSoc;
    uint32_t iVoltage;
    uint32_t iCurrent;
    uint32_t iTemp;
    uint32_t iRange;
    uint32_t iPower;
}sBatteryInfo_t;

typedef struct
{
	uint8_t iDayId;
	uint8_t iDay;
	uint8_t iMonth;
	uint8_t iYear;
	uint8_t iHour;
	uint8_t iMinute;
	uint8_t iSecond;
	uint8_t iIsEdit;
	uint8_t iState;
	bool    bPM;
	bool 	bEnable;
}sTimeInfo_t;

typedef enum
 {
	 eStateNone		=  -1,
	 eStateDay    	=  0,
	 eStateMonth 	=  1,
	 eStateYear  	=  2,
	 eStateHour 	=  3,
	 eStateMinute  	=  4,
	 eStateMeridiem =  5,
	 eStateMax =  5,
 }eRtcState_t;

typedef struct
{
	int32_t iId;
	int32_t iStatus;
}sDeviceStatus_t;

typedef struct
{
    int32_t iEventId;
    int8_t sData[252];
}sMessage_t;

typedef struct
{
    QueueHandle_t 		QueueId;
}sController_t;

typedef enum
{
	eDevData        =   1,
	eAuthenData   	=   2,
	eCallResp       =   3,
    eBtTest         =   4
}sMsgType;

typedef struct
{
	uint32_t	iOdoVal;
	uint32_t 	iCurTrip;
}ClusterInfo_t;

typedef struct
{
	uint32_t  iSlNum;
	char  sVin[32];
}sDeviceInfo_t;

typedef struct
{
	uint16_t  iSwVersion;
	uint16_t  iHwVersion;
	uint32_t  iSlNum;
	char      sVin[32];
	char      sDate[32];
} sSystemInfo_t;

typedef enum
{
	eAlert_CAN        		 = 0x100,
	eAlert_Rtc          	 = 0x101,
	eAlert_Switch    		 = 0x102,
	eAlert_Trip    			 = 0x103,
	eAlert_Custom  			 = 0x104,
}eAlertId_t;

typedef struct
{
	eAlertId_t     iId;
	uint32_t 	iCode;
//	int8_t     sMsg[128];
	uint8_t    iCount;
}sAlertInfo_t;

typedef enum
{
	eCAN_Working			 = 0,
	eCAN_NoData         	 = 1,
	eCAN_Error         		 = 2,
	eCAN_SpeedDataTimeOut    = 3,
}eAlertCANCode_t;

typedef enum
{
	eRTC_Working			 = 0,
	eRTC_NotWorking          = 1,
	eRTC_NotSetOrBtryWeak    = 2,
}eAlertRTCCode_t;

// customer specific definations
typedef enum
{
	eCANID_Frame1   =  0x301,  //SWTelltales
	eCANID_Frame2   =  0x302,  //TextAlert
//	eCANID_Frame3   =  0x303,  //ODO1
//	eCANID_Frame4   =  0x304,  //
	eCANID_Frame7   =  0x502,  //BatTemp
	eCANID_Frame8   =  0x505,  //Charging
	eCANID_Frame9   =  0x306,  //Odo and trip 
	eCANID_Internal =  0x195,  // Internal
	eCANID_UDSRx    =  0x200,  // UDS Rx
	eCANID_UDSTx    =  0x201,  // UDS Tx
}eCANFrame_t;


typedef enum
{
	eCharging_Notactive  =0,
	eCharging_Completed   = 1,
	eCharging_InProgress = 2,
	eCharging_Failed  = 3,
}eChargeStatus_t;

typedef enum
{
	ePowerSteering_Notactive  	= 0,
	ePowerSteering_Working  	= 1,
	ePowerSteering_Failed   	= 2,
}ePowSteering_t;


typedef struct
{

	bool bHvbattery;
	bool bCoolanttemp;
	bool bBmsfault;
	bool bThernalrunaway;
	uint8_t iDriveMode;
	uint16_t  iCount;
}sStatus0_t;

typedef struct
{
	bool bRegen;
	bool bMaxspeed;
	bool bLvbatttery;
	uint8_t iCharge;
	uint8_t iSteering;
	uint16_t  iCount;
}sStatus1_t;

typedef struct
{
	bool bLowcoolant;
	bool bBreakfault;
	bool bParking;
	bool bAirbag;
	bool bEscape;
	bool bRearfog;
	bool bRevive;
	uint16_t  iCount;
}sStatus2_t;

typedef struct
{
	bool bReadytoDrive;
	bool bDriveDisabled;
	bool bServiceDisconnect;
	bool bHVBatteryFault;
	bool bLowSoc;
	bool bBatCoolantTemp;
	bool bLimpHome;
	bool bHillHold;
	uint16_t  iCount;
}sStatus3_t;

#ifdef __cplusplus
}
#endif


#endif /* CLUSTER_COMMON_H_ */
