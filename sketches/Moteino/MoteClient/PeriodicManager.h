#ifndef PeriodicManager_h
#define PeriodicManager_h

#if ARDUINO < 100
#include <WProgram.h>
#include <pins_arduino.h>  // fix for broken pre 1.0 version - TODO TEST
#else
#include <Arduino.h>
#endif

class PeriodicManager
{
public:
	PeriodicManager() {};

	void addPeriodicTask(void (*task)());
	void run();

private:
	void (*p[4])();		// Function pointers
};

#ifdef WITH_RFM69
	radio.initialize(FREQUENCY,NODEID,NETWORKID);
	radio.sleep();
#endif

#endif
//
// END OF FILE
//
