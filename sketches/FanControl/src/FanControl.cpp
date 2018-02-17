// Arduino libraries
#include "Arduino.h"
// #include <SoftwareSerial.h>
//#include <Wire.h>          //comes with Arduino IDE (www.arduino.cc)

// Custom Arduino libraries
#include "FanControl.h"

#include <DigiUSB.h>

// #include "MemoryFree.h"
//#include "dht.h"
//#include "LowPower.h"      // https://github.com/rocketscream/Low-Power
//#include "RTClib.h"        // https://github.com/adafruit/RTClib/tree/1.2.0

/*-------------------------------- Defines -----------------------------------*/

/*------------------------------ Data Types ---------------------------------*/

/*-------------------------- Routine Prototypes ------------------------------*/
void InitIOPins();

/* ---------------------------- Global Variables ---------------------------- */

// ========================== End of Header ================================= //

/* -------------------------------- Routines -------------------------------- */
void InitIOPins()
{
//    SET_DIGITAL_PINS_AS_INPUTS();

    // -- Custom IO setup --
    pinMode(INFO_LED, OUTPUT);
    pinMode(TEMP_LM35, INPUT);
}

void setup()
{
    DigiUSB.begin();

    /* IO Pins need to be initialized prior to other peripherals start */
    InitIOPins();
}

void loop()
{
//    LED_ON;
//    delay(50);
//    LED_OFF;
//    delay(50);
    
    static long long timer = millis() + 1000;
    
    if(millis() > timer) {
        timer += 1000;
        
        float cel = ReadTemp();
//        DEBUGLN(cel);
    }
    
    DigiUSB.delay(10);
}

float ReadTemp()
{
    float val = analogRead(1);
    
    int mv = map(val, 0,1023, 0, 5000);
    
    float cel = (val*500) / 1023;
    
    DEBUGLN(mv);
    
    return cel;
}