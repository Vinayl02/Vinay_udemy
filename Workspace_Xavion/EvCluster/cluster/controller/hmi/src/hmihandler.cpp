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
    		motorInfoChanged((int)sParam.iSpeed, (int)sParam.iRange,
    						 (int)sParam.iTrip,  (int)sParam.iOdo,
							 (int)sParam.iRpm,	 (int)sParam.iPower,
							 sParam.bRegen);
    	}
        break;
    	case Event_Battery:
    	{
    		sBatteryInfo_t sParam = {0};
    		memcpy((char *)(&sParam) ,(char *)(pEventParam),sizeof(sBatteryInfo_t));
    		battInfoChanged((int)sParam.iSoc,(int)sParam.iTemp);
    	}
        break;
    	case Event_Indicator:
    	{
    		sStatus_t sParam =  {0};
    		memcpy((char *)(&sParam),(char *)(pEventParam),sizeof(sStatus_t));
    		statusInfoChanged(	sParam.bBatFault,sParam.bBatMalFunc,
    				            sParam.bPTFault,sParam.bPTMalFunc,
								sParam.bHvil,sParam.bCoolant);
    	}
        break;
    	case Event_HwIndicator:
    	{
    		sHwInputs_t sParam = {0};
    		memcpy((char *)(&sParam),(char *)(pEventParam),sizeof(sHwInputs_t));
    		hwStatusChanged(sParam.bInput2,sParam.bInput3,
    				        sParam.bInput4,sParam.bInput5,
							sParam.bInput6,sParam.bInput7,
							sParam.bInput8,sParam.bInput9,
							sParam.bInput10);
    	}
        break;
    	case Event_DriveMode:
    	{
    		sDriveMode_t sParam = {0};
    		memcpy((char *)(&sParam),(char *)(pEventParam),sizeof(sDriveMode_t));
    		driveModeChanged(sParam.iDriveVal,sParam.iGearVal);
    	}
        break;	

    	case Event_Fault:
    	{
    		sFault_t sParam = {0};
    		memcpy((char *)(&sParam),(char *)(pEventParam),sizeof(sFault_t));
    		switch(sParam.iId)
    		{
				case 0:
					fault0StatusChanged((int)(sParam.ifault0),(int)(sParam.ifault1),
										(int)(sParam.ifault2),(int)(sParam.ifault3));
				break;
	    		case 1:
	    			fault1StatusChanged((int)(sParam.ifault0),(int)(sParam.ifault1),
										(int)(sParam.ifault2),(int)(sParam.ifault3));
	    		break;
	    		case 2:
	    			fault2StatusChanged((int)(sParam.ifault0),(int)(sParam.ifault1),
										(int)(sParam.ifault2),(int)(sParam.ifault3));
	    		break;
	    		case 3:
	    			fault3StatusChanged((int)(sParam.ifault0),(int)(sParam.ifault1),
										(int)(sParam.ifault2),(int)(sParam.ifault3));
	    		break;
	    		case 4:
	    			fault4StatusChanged((int)(sParam.ifault0),(int)(sParam.ifault1),
										(int)(sParam.ifault2),(int)(sParam.ifault3));
	    		break;
	    		default:
	    		break;
    		}

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
    		sChargeInfo_t sParam = {0};
    		memcpy((char *)(&sParam),(char *)(pEventParam),sizeof(sChargeInfo_t));
    		chargerInfoChanged(sParam.bType,sParam.bStatus,sParam.bGunlock,(int)(sParam.iTime));
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
