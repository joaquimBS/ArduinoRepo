#ifndef global_h
#define global_h

#include "Arduino.h"

// I/O Pins
enum {P0, P1, P2, INT_RED, DHT_PIN, P5, P6, P7, FLASH_SS, INFO_LED, P10, P11, P12, P13, PIN_COUNT};	/* Moteino */

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
