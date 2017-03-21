#ifndef global_h
#define global_h

#include "Arduino.h"

// I/O Pins
enum {P0, P1, P2_RESERVED, INT_RED, DHT_PIN, OLED_VCC, P6, P7, FLASH_SS, INFO_LED, P10, P11, P12, P13, PIN_COUNT};	/* Moteino */

#define SET_DIGITAL_PINS_AS_INPUTS()        \
        uint8_t p;                          \
        for(p=0; p<PIN_COUNT; p++) {        \
            pinMode(p, INPUT);              \
            digitalWrite(p, LOW);           \
        } while(0)  // This while is to allow ';' at the end of the macro


#define DEBOUNCE_TIME_MS    200
#define PB_PRESSED          LOW
#define PB_RELEASED         HIGH

#define LED_ON      digitalWrite(INFO_LED, HIGH)
#define LED_OFF     digitalWrite(INFO_LED, LOW)

#define USE_DEBUG

#if defined(USE_DEBUG)
    #define DEBUGG(str)   Serial.print(str)
    #define DEBUGLN(str) Serial.println(str)
#else
    #define DEBUG(str)
    #define DEBUGLN(str)
#endif

#endif /* global_h */
