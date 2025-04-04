#include <hmihandler.h>
#include "fsl_debug_console.h"  //debug console.
#include "cluster_common.h"
HmiHandler::HmiHandler()
{

}

void HmiHandler::updatePlatform(int eEvent, void* pEventParam)
{

}

void HmiHandler::updateUI(int eEvent, void* pEventParam)
{
    switch ((ClusterEvents_t)(eEvent))
    {
    	case Event_Motor:
    	{
    		sMotorInfo_t  sParam = {0};
    		memcpy((char *)(&sParam),(char*)(pEventParam),sizeof(sMotorInfo_t));
    		motorInfoChanged((int)sParam.iSpeed,(int)sParam.iOdo,(int)sParam.iTrip);
    	}
        break;
    	case Event_Battery:
    	{
    		sBatteryInfo_t  sParam = {0};
    		memcpy((char *)(&sParam),(char*)(pEventParam),sizeof(sBatteryInfo_t));
    		battInfoChanged((int)sParam.iSoc, (int)sParam.iRange, (int)sParam.iTemp);
    	}
        break;

    	case Event_Indicator:
    	{
    		sStatus0_t  sParam = {0};
    		memcpy((char *)(&sParam),(char*)(pEventParam),sizeof(sStatus0_t));
    		status0InfoChanged(	sParam.bHvbattery,sParam.bCoolanttemp,
								sParam.bBmsfault,sParam.bThernalrunaway,
								sParam.iDriveMode);
    	}
        break;

    	case Event_Indicator1:
    	{
    		sStatus1_t  sParam = {0};
    		memcpy((char *)(&sParam),(char*)(pEventParam),sizeof(sStatus1_t));
    		status1InfoChanged(	sParam.bRegen,sParam.bMaxspeed,
								sParam.bLvbatttery,sParam.iCharge,
								sParam.iSteering);

    	}
        break;
    	case Event_Indicator2:
    	{
    		sStatus2_t  sParam = {0};
    		memcpy((char *)(&sParam),(char*)(pEventParam),sizeof(sStatus2_t));
    		status2InfoChanged(sParam.bLowcoolant,sParam.bBreakfault,
							   sParam.bParking,sParam.bAirbag);
    	}
        break;
    	case Event_Indicator3:
    	{
    		sStatus3_t  sParam = {0};
    		memcpy((char *)(&sParam),(char*)(pEventParam),sizeof(sStatus3_t));

    		status3InfoChanged(	sParam.bReadytoDrive,
    							sParam.bDriveDisabled,
								sParam.bServiceDisconnect,
								sParam.bHVBatteryFault,
								sParam.bLowSoc,
								sParam.bBatCoolantTemp,
								sParam.bLimpHome,
								sParam.bHillHold);
    	}
        break;
    	case Event_HwIndicator:
    	{
    		sHwInputs_t  sParam = {0};
    		memcpy((char *)(&sParam),(char*)(pEventParam),sizeof(sHwInputs_t));
    		hwStatusChanged(sParam.bInput1,sParam.bInput2,
    					    sParam.bInput3,sParam.bInput4,
							sParam.bInput5,sParam.bInput6,
							sParam.bInput7,sParam.bInput8,
							sParam.bInput9);
    	}
        break;
    	case Event_NavSwitch:
    	{
    		sNavSwitch_t  sParam = {0};
    		memcpy((char *)(&sParam),(char*)(pEventParam),sizeof(sNavSwitch_t));
     		navSwitchStatusChanged(sParam.iVal);
    	}
        break;
    	case Event_Charge:
    	{
    		sChargeInfo_t  sParam = {0};
    		memcpy((char *)(&sParam),(char*)(pEventParam),sizeof(sChargeInfo_t));
    		chargeInfoChanged((int)sParam.iTime, (int)sParam.iEnergyCon);
    	}
        break;
    	case Event_Time:
		{
    		sTimeInfo_t sParam = {0};
    		memcpy((char *)(&sParam),(char *)(pEventParam),sizeof(sTimeInfo_t));
    		dateTimeChanged((int)sParam.iDay,(int)sParam.iMonth,(int)sParam.iYear,
    						(int)sParam.iHour,(int)sParam.iMinute, sParam.bPM,
							(int)sParam.iIsEdit,(int)sParam.iState);
    	}
        break;
    	case Event_Alert:
		{
    		sAlertInfo_t sParam;
    		memcpy((char *)(&sParam),(char *)(pEventParam),sizeof(sAlertInfo_t));
    		alertInfoChanged((int)sParam.iId,(int)sParam.iCode);
    	}
    	break;
    	case Event_DeviceInfo:
    	{
    		sSystemInfo_t sParam = {0};
    		memcpy((char *)(&sParam),(char *)(pEventParam),sizeof(sSystemInfo_t));
    		deviceInfoChanged((int)sParam.iSwVersion,
    						  (int)sParam.iHwVersion,
							  (int)sParam.iSlNum);
    	}
    	break;
    	default:
        break;
    }
}

void HmiEventQueue::onEvent(const HmiEvent &sEvent)
{
	HmiHandler::instance().updateUI(sEvent.command,sEvent.pData);
}

static HmiEventQueue hEventQueue;


void UI::sendToUI(int iEventVal, void* pEventParam)
{
	HmiEvent sEvent;
	sEvent.command = iEventVal;
	sEvent.pData = pEventParam;
    hEventQueue.postEvent(sEvent);

}
