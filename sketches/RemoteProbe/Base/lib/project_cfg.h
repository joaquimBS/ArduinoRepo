#ifndef global_h
#define global_h

#include "Arduino.h"

#define SET_DIGITAL_PINS_AS_INPUTS()        \
        uint8_t p;                          \
        for(p=0; p<DIGITAL_PIN_COUNT; p++) {\
            pinMode(p, INPUT);              \
            digitalWrite(p, LOW);           \
        } while(0)  // This while is to allow ';' at the end of the macro

#define DELTA_TIME(in_time) (millis() - in_time)

// I/O Digital Pins
enum {
    D0 = (uint8_t)0, /* RX */
    D1, /* TX */
    D2_RESERVED_RADIO,
    INT_RED,
    D4,
    D5,
    D6,
    SS_RX,
    SS_TX,
    INFO_LED, /* BUILT-IN LED */
    D10_RESERVED_RADIO,
    D11_RESERVED_RADIO, /* MOSI */
    D12_RESERVED_RADIO, /* MISO */
    D13_RESERVED_RADIO, /* SCK */
    D14,
    D15,
    D16,
    D17,
    D18,
    D19,
    DIGITAL_PIN_COUNT
}; /* Moteino */

#define DEBOUNCE_TIME_MS    100
#define PB_PRESSED          LOW
#define PB_RELEASED         HIGH

#define NULL_PTR (void*)0

#define LED_ON      digitalWrite(INFO_LED, HIGH)
#define LED_OFF     digitalWrite(INFO_LED, LOW)

#define USE_DEBUG
#define SERIAL_BR 115200

#if defined(USE_DEBUG)
#define DEBUG(str)   Serial.print(str)
#define DEBUGVAL(str, val) \
        Serial.print(__func__); \
        Serial.print("["); Serial.print(__LINE__); Serial.print("] "); \
        Serial.print(str); Serial.println(val); 
#define DEBUGLN(str) \
        Serial.print(__func__); \
        Serial.print("["); Serial.print(__LINE__); Serial.print("] "); \
        Serial.println(str)
#else
#define DEBUG(str)
#define DEBUGLN(str)
#define DEBUGVAL(str, val)
#endif

#endif /* global_h */
