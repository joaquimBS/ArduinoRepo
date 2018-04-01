// Arduino libraries
// #include <SoftwareSerial.h>
//#include <SPI.h>           //comes with Arduino IDE (www.arduino.cc)
//#include <Wire.h>

#include "main.h"

// Custom Arduino libraries
//#include "LowPower.h"

/*-------------------------------- Defines -----------------------------------*/
#define APPNAME_STR "AppName"
#define BUILD_STR "1.0"

/*------------------------------- Data Types ---------------------------------*/

/*-------------------------- Routine Prototypes ------------------------------*/
void Rsi1();
void InitIOPins();
void GoToSleep(uint16_t seconds);

/* ---------------------------- Global Variables ---------------------------- */

/* -------------------------------- Routines -------------------------------- */
void Rsi1()
{
    // TBD
}

void InitIOPins()
{
    SET_DIGITAL_PINS_AS_INPUTS();

    // -- Custom IO setup -- //
}

void setup()
{
#if defined(USE_DEBUG)
    Serial.begin(SERIAL_BR);
    while (!Serial);

    DEBUGVAL("AppName=", APPNAME_STR);
    DEBUGVAL("AppVersion=", BUILD_STR);
#endif

    /* IO Pins need to be initialized prior to other peripherals start */
//    InitIOPins();

    LED_ON;

//  Initialization code  
    
    LED_OFF;
}

void loop()
{
    LED_ON;
//    GoToSleep(1);
    delay(200);
    LED_OFF;
//    GoToSleep(1);
    delay(200);
}

void GoToSleep(uint16_t seconds)
{
//    digitalWrite(SDA, LOW);  // To minimize I2C current consumption during sleep
//    digitalWrite(SCL, LOW);
//
//#ifdef USE_DEBUG
//    /* To allow DEBUG traces to finish Tx */
//    delay(50);
//#endif
//    
//    // Enter power down state with ADC and BOD module disabled.
//    // Wake up when wake up pin is low.
//    while(seconds-- > 0) {
//        LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
//        /* ZZzzZZzzZZzz */
//    }
}
