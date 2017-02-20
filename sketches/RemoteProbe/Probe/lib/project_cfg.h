#ifndef global_h
#define global_h

#include "Arduino.h"

// I/O Pins
enum {D0, D1, D2, MIC_VCC, D4, D5, DHT_PIN, D7, FLASH_SS, INFO_LED, D10, D11, D12, D13, PIN_COUNT};	/* Moteino */

#define LED_ON      digitalWrite(INFO_LED, HIGH)
#define LED_OFF     digitalWrite(INFO_LED, LOW)

// #define USE_DEBUG

#if defined(USE_DEBUG)
    #define DEBUGG(str)   Serial.print(str)
    #define DEBUGLN(str) Serial.println(str)
#else
    #define DEBUG(str)
    #define DEBUGLN(str)
#endif

#endif /* global_h */
