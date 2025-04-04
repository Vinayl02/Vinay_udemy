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

	Qul::Signal<void(int mfault0,int mfault1,int mfault2,int mfault3)>fault0StatusChanged;
	Qul::Signal<void(int mfault0,int mfault1,int mfault2,int mfault3)>fault1StatusChanged;
	Qul::Signal<void(int mfault0,int mfault1,int mfault2,int mfault3)>fault2StatusChanged;
	Qul::Signal<void(int mfault0,int mfault1,int mfault2,int mfault3)>fault3StatusChanged;
	Qul::Signal<void(int mfault0,int mfault1,int mfault2,int mfault3)>fault4StatusChanged;
	Qul::Signal<void(int mday,int mmonth,int myear,int mhour,
					 int mminute, bool mPM, int medit,int mstate)>dateTimeChanged;
	Qul::Signal<void(int mkey)>navSwitchStatusChanged;
	Qul::Signal<void(int msoc, int mbattemp)> battInfoChanged;
	Qul::Signal<void(bool mtype, bool mstatus,bool mgunlock,int mctime)> chargerInfoChanged;
    Qul::Signal<void(int mdrivemode,int mgearmode)>driveModeChanged;
    Qul::Signal<void(int mspeed,int mrange,int mtrip, int modo,int mrpm,int mpower, bool mregen)> motorInfoChanged;
    Qul::Signal<void(int mid,int mcode)> alertInfoChanged;
    Qul::Signal<void(bool mbatfault,bool mbatmalfunc,bool mptfault,bool mptmalfunc,bool mhvil,bool mcoolent)>statusInfoChanged;
    Qul::Signal<void(bool mleftind,bool mrightind, bool mhighbeam,bool mlowbeam,bool mhazard,
					 bool mbrakefailed, bool msafetybelt,bool mhandbrake,bool mfoglight)>hwStatusChanged;
    Qul::Signal<void(int mmsgid)>messageChanged;
	Qul::Signal<void(int mswversion ,int mhwversion ,int mslnum)>deviceInfoChanged;
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
