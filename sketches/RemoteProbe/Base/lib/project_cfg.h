#ifndef global_h
#define global_h

#include "Arduino.h"

// I/O Pins
enum {P0, P1, P2_RESERVED, INT_RED, DHT_PIN, OLED_VCC, P6, P7, SS_RX, SS_TX, P10, P11, P12, INFO_LED, PIN_COUNT};	/* Moteino */

#define SET_DIGITAL_PINS_AS_INPUTS()        \
        uint8_t p;                          \
        for(p=0; p<PIN_COUNT; p++) {        \
            pinMode(p, INPUT);              \
            digitalWrite(p, LOW);           \
        } while(0)  // This while is to allow ';' at the end of the macro

#define DELTA_TIME(in_time) (millis() - in_time)

#define SERIAL_BR 115200

#define DEBOUNCE_TIME_MS    100
#define PB_PRESSED          LOW
#define PB_RELEASED         HIGH

#define LED_ON      digitalWrite(INFO_LED, HIGH)
#define LED_OFF     digitalWrite(INFO_LED, LOW)

#define USE_DEBUG

#if defined(USE_DEBUG)
    #define DEBUG(str)   Serial.print(str)
    #define DEBUGLN(str) Serial.println(str)
#else
    #define DEBUG(str)
    #define DEBUGLN(str)
#endif

#endif /* global_h */
