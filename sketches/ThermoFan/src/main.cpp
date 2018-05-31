// Arduino libraries
//#include <SoftwareSerial.h>
//#include <SPI.h>           //comes with Arduino IDE (www.arduino.cc)
//#include <Wire.h>

#include "main.h"

// Custom Arduino libraries
//#include "LowPower.h"
#include "dht.h"
//#include <DigiUSB.h>

/*-------------------------------- Defines -----------------------------------*/
#define APPNAME_STR "AppName"
#define BUILD_STR "1.0"

/*------------------------------- Data Types ---------------------------------*/

/*-------------------------- Routine Prototypes ------------------------------*/
void Rsi1();
void InitIOPins();
void FanLogic();
void GoToSleep(uint16_t seconds);

/* ---------------------------- Global Variables ---------------------------- */
dht DHT;

/* -------------------------------- Routines -------------------------------- */
void Rsi1()
{
    // TBD
}

void InitIOPins()
{
//    SET_DIGITAL_PINS_AS_INPUTS();

    // -- Custom IO setup -- //
    pinMode(INFO_LED, OUTPUT);
    pinMode(RELAY_TRIGGER, OUTPUT);
    
    digitalWrite(RELAY_TRIGGER, HIGH);
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
    InitIOPins();

//    LED_ON;

//  Initialization code
    DigiUSB.begin();
//    RELAY_OFF;
    
//    LED_OFF;
}

void loop()
{
    LED_ON;
    DigiUSB.delay(1000);

    LED_OFF;
    DigiUSB.delay(1000);

    // DISPLAY DATA
    DigiUSB.print(DHT.humidity, 1);
    DigiUSB.print(",\t");
    DigiUSB.println(DHT.temperature, 1);

    FanLogic();
}

#define TEMP_THRESHOLD 42
#define HYSTERESIS 3

void FanLogic()
{
    static bool fan_state = false;
    int chk = DHT.read22(DHT22_PIN);
    
    if(0 == chk) {
        if(fan_state == true) {
            if(DHT.temperature < (TEMP_THRESHOLD - HYSTERESIS)) {
                fan_state = false;
                digitalWrite(RELAY_TRIGGER, LOW);
                delay(250);
                digitalWrite(RELAY_TRIGGER, HIGH);
            }
        }
        else if(fan_state == false) {
            if(DHT.temperature > (TEMP_THRESHOLD + HYSTERESIS)) {
                fan_state = true;
                digitalWrite(RELAY_TRIGGER, LOW);
                delay(250);
                digitalWrite(RELAY_TRIGGER, HIGH);
            }
        }
    }
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
