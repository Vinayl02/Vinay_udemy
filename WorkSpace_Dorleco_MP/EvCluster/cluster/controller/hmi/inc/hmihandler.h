#ifndef HMIHANDLER_H
#define HMIHANDLER_H

#include <qul/singleton.h>
#include <qul/property.h>
#include <qul/eventqueue.h>

struct HmiHandler : public Qul::Singleton<HmiHandler>
{
    friend struct Qul::Singleton<HmiHandler>;
	
	enum ClusterEvents_t
	{
	    Event_None				= 0,
		Event_Motor				= 1,
		Event_Battery			= 2,
		Event_DriveMode			= 3,
		Event_DeviceStatus		= 4,
		Event_Indicator			= 5,
		Event_HwIndicator		= 6,
		Event_Message			= 7,
		Event_Standby			= 8,
		Event_Fault				= 9,
		Event_Call				= 10,
		Event_TurnByTurnNav		= 11,
		Event_NavSwitch			= 12,
		Event_Time              = 13,
		Event_Alert				= 14,
		Event_DeviceInfo        = 15,
		Event_BTMessage         = 16,
		Event_Charge			= 17,
	//customer specific
		Event_BMS              	= 18,
		Event_Indicator1		= 19,
		Event_Indicator2		= 20,
		Event_Indicator3		= 21,
		Event_Max				= 22,
	};

	Qul::Signal<void(int mday,int mmonth,int myear,int mhour,int mminute, bool mPM, int medit,int mstate)>dateTimeChanged;
	Qul::Signal<void(int mkey)>navSwitchStatusChanged;
	Qul::Signal<void(int mmsgid)>messageChanged;
    Qul::Signal<void(bool mignition,bool mleftind,bool mrightind, bool mhighbeam,
    		         bool mlowbeam,bool	mdoorind,bool mposition, bool mseatybelt,
					 bool mfoglight)>hwStatusChanged;
    Qul::Signal<void(int mid,int mcode)> alertInfoChanged;
    Qul::Signal<void(int mswversion ,int mhwversion ,int mslnum)>deviceInfoChanged;
    //customer specific
    Qul::Signal<void(int mspeed,int modo, int mtrip)> motorInfoChanged;
    Qul::Signal<void(int msoc,int mrange, int mbattemp)> battInfoChanged;
    Qul::Signal<void(int menergyavg,int menergycon)> bmsInfoChanged;
    Qul::Signal<void(bool mhvbattery,bool mcoolanttemp, bool mbmsfault,bool mthernalrunaway,int	mdrive)>status0InfoChanged;
    Qul::Signal<void(bool mregen,bool mmaxspeed, bool mlvbatttery,int mcharge,int msteering)>status1InfoChanged;
    Qul::Signal<void(bool mlowcoolant,bool mbreakfault, bool mparking,bool mairbag)>status2InfoChanged;
    Qul::Signal<void(bool mreadytodrive,bool mdrivedisabled, bool mserdisc,bool mhvbatfault,
			 	 	 bool mlowsoc,bool mbatlowcooltemp, bool mlimphome,bool mhillhold)>status3InfoChanged;

    Qul::Signal<void(int mtime,int menergycon)> chargeInfoChanged;

    void updatePlatform(int eEvent, void* pEventParam);
    void updateUI(int eEvent, void* pEventParam);

private:
    HmiHandler();
    HmiHandler(const HmiHandler &);
    HmiHandler &operator=(const HmiHandler &);
};

struct HmiEvent
{
	int command;
    void* pData;
};

class HmiEventQueue : public Qul::Singleton<HmiEventQueue>, public Qul::EventQueue<HmiEvent, Qul::EventQueueOverrunPolicy_Discard ,250>
{
public:
    void onEvent(const HmiEvent &sEvent);
    void onQueueOverrun() override
    {
        clearOverrun();
        Qul::PlatformInterface::log("\r\nHMI receiver queue overrun\r\n");
    }

    void onEventDiscarded(HmiEvent * &event)
    {
    	Qul::PlatformInterface::log("\r\nonEventDiscarded\r\n");
    }
};

namespace UI {
void sendToUI(int eEvent, void* pEventParam);
}

#endif // HMIHANDLER_H
