#ifndef global_h
#define global_h

#include <Arduino.h>

// I/O Pins
enum {D0, D1, INTERRUPT_PIN, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12, INFO_LED, PIN_COUNT};	/* Moteino */

/* Custom defines */
#define SERIAL_BR 115200

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
