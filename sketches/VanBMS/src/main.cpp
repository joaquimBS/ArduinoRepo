// Arduino libraries
// #include <SoftwareSerial.h>
//#include <SPI.h>           //comes with Arduino IDE (www.arduino.cc)
//#include <Wire.h>

#include <stdint.h>

#include "main.h"

// Custom Arduino libraries
//#include "LowPower.h"

/*-------------------------------- Defines -----------------------------------*/
#define APPNAME_STR "VanBMS"
#define BUILD_STR "1.0"

/* Assumption. RELAY_TRIGGER is already HIGH */
#define RELAY_PULSE do {digitalWrite(RELAY_TRIGGER, LOW); \
                        delay(250); \
                        digitalWrite(RELAY_TRIGGER, HIGH); \
                        relay_pulses++; \
                    } while(0)

/*------------------------------- Data Types ---------------------------------*/

/*-------------------------- Routine Prototypes ------------------------------*/
void Rsi1();
void InitIOPins();
void GoToSleep(uint16_t seconds);

static uint16_t SenseCarVBatMv(void);
static uint16_t SenseSecondVBatMv(void);

/* ---------------------------- Global Variables ---------------------------- */
uint16_t relay_pulses = 0;

/* -------------------------------- Routines -------------------------------- */
void Rsi1()
{
    // TBD
}

void InitIOPins()
{
    SET_DIGITAL_PINS_AS_INPUTS();

    // -- Custom IO setup -- //
    pinMode(RELAY_TRIGGER, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    
    pinMode(CAR_VBAT, INPUT_PULLUP);
    pinMode(SECOND_VBAT, INPUT_PULLUP);
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

    LED_ON;

//  Initialization code
    relay_pulses = 0;
    digitalWrite(RELAY_TRIGGER, HIGH); // Important
    
    LED_OFF;
}

void loop()
{
    LED_ON;
    RELAY_PULSE;
    delay(2000);

    LED_OFF;
    RELAY_PULSE;
    delay(2000);
    
    DEBUGVAL("relay_pulses=", relay_pulses);
    DEBUGVAL("SenseCarVBatMv=", SenseCarVBatMv());
    DEBUGVAL("SenseSecondVBatMv=", SenseSecondVBatMv());
}

static uint16_t SenseCarVBatMv()
{
    int car = analogRead(CAR_VBAT);
    
    const float r_div = ((9930.0 + 4680.0) / 4680.0); // real measurements 
    
    return map(car, 0, 1023, 0, 4999) * r_div;
}

static uint16_t SenseSecondVBatMv()
{
    int second = analogRead(SECOND_VBAT);
    
    const float r_div = ((9900.0 + 4650.0) / 4650.0); // real measurements 
    
    return map(second, 0, 1023, 0, 4999) * r_div;
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
