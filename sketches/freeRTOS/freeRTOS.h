#ifndef FREERTOS_H
#define FREERTOS_H

// Custom Arduino libaries
#include <DHT.h>
#include "RTClib.h"			// https://github.com/adafruit/RTClib
// #include "IRremote.h"		// https://github.com/z3t0/Arduino-IRremote
#include <Arduino_FreeRTOS.h>
#include <avr/power.h>
#include <avr/sleep.h>
// #include "semphr.h"  // add the FreeRTOS functions for Semaphores (or Flags).

// Pinout
// #define PIN0  0		// Rx
// #define PIN1  1		// Tx
#define PIN_BTN_YLW 	2	// INT0
#define PIN_BTN_RED		3 	// INT1
#define DHTPIN			4
// #define DHT22_VCC 		5
// #define PIN_BTN_LESS	6
// #define RTC_VCC			7
// #define RELE_PIN		8
// #define INFO_LED		9
// #define PIN10			10

#define INFO_LED		12



#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321


// OLED region
#define I2C_ADDRESS 0x3C
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"

#endif //FREERTOS_H
