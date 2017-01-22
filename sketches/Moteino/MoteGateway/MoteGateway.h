#ifndef MoteGateway
#define MoteGateway

// AVR Libraries
// #include <avr/power.h>
// #include <avr/sleep.h>

// Arduino libraries
// #include <SoftwareSerial.h>

// Custom Arduino libraries
// #include "RTClib.h"         // https://github.com/adafruit/RTClib
// #include "IRremote.h"       // https://github.com/z3t0/Arduino-IRremote
// #include <Arduino_FreeRTOS.h>
// #include "semphr.h"  // add the FreeRTOS functions for Semaphores (or Flags).
// #include <MemoryFree.h>
#include <dht.h>
#include "LowPower.h"       // https://github.com/rocketscream/Low-Power
// #include <Sleep_n0m1.h>
#include "PeriodicManager.h"

#define WITH_RFM69
#define WITH_SPIFLASH
#define WITH_OLED

// I/O Pins
enum {P0, P1, P2, INT_RED, DHT_PIN, OLED_ENABLE, P6, P7, FLASH_SS, INFO_LED, P10, P11, P12, P13, PIN_COUNT};	/* Moteino */
// enum {P0, P1, P2, P3, DHT, OLED_ENABLE, P6, P7, P8, P9, P10, P11, P12, INFO_LED , PIN_COUNT};	/* Arduino Pro */

#if defined(WITH_RFM69) || defined(WITH_SPIFLASH)
  #include <SPI.h>                //comes with Arduino IDE (www.arduino.cc)
  #if defined(WITH_RFM69)
    #include <RFM69.h>            //get it here: https://www.github.com/lowpowerlab/rfm69
    RFM69 radio;
    #define GATEWAYID   1
    #define NETWORKID 100
    #define NODEID 123
    #define FREQUENCY RF69_868MHZ
  #endif
  #if defined(WITH_SPIFLASH)
    #include <SPIFlash.h>         //get it here: https://www.github.com/lowpowerlab/spiflash
    SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)
  #endif
#endif

#ifdef WITH_OLED
	#define I2C_ADDRESS 0x3C
	#include "SSD1306Ascii.h"
	#include "SSD1306AsciiAvrI2c.h"

	SSD1306AsciiAvrI2c oled;
#endif

typedef enum {CYCLIC, INT_EXT} t_wake_up_cause;
volatile t_wake_up_cause wake_up_cause;

// Custom defines
#define SERIAL_BR 115200
#define DHTTYPE   DHT22   // DHT 22  (AM2302), AM2321
dht DHT;

void periodicTask();
void RSI_Red();
void goToSleep();
void aboutToStart();
void updateOledData();

#endif // MoteGateway
