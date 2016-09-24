// AVR Libraries
// #include <avr/power.h>
// #include <avr/sleep.h>

// Arduino libraries
// #include <SoftwareSerial.h>

// Custom Arduino libraries
// #include "RTClib.h"			    // https://github.com/adafruit/RTClib
// #include "IRremote.h"			// https://github.com/z3t0/Arduino-IRremote
// #include <Arduino_FreeRTOS.h>
// #include "semphr.h"  // add the FreeRTOS functions for Semaphores (or Flags).
// #include <dht.h>
// #include <MemoryFree.h>

// I/O Pins
enum {P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13_INFO_LED, PIN_COUNT};

// Custom defines
#define SERIAL_BAUDRATE 	19200

#define FREQ_1000_MS  1000
#define FREQ_500_MS   500
#define FREQ_100_MS   100

unsigned long ticks_1000 = 0;
unsigned long ticks_500  = 0;
unsigned long ticks_100  = 0;

void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(SERIAL_BAUDRATE);
  while(!Serial);

  for(int p=0; p<PIN_COUNT; p++) {
  	pinMode(p, INPUT_PULLUP);
  }

  pinMode(P13_INFO_LED, OUTPUT);

  ticks_1000 = millis();
  ticks_500  = millis();
  ticks_100  = millis();
}

void loop()
{
	static bool led_state = true;

	if((millis() - ticks_1000) >= FREQ_1000_MS) {
		ticks_1000 = millis();
		/*
		 * So something every 1000 ms
		 */
		 digitalWrite(P13_INFO_LED, led_state);
		 led_state = !led_state;
	}

	if((millis() - ticks_500) >= FREQ_500_MS) {
		ticks_500 = millis();
		/*
		 * So something every 500 ms
		 */
	}

	if((millis() - ticks_100) >= FREQ_100_MS) {
		ticks_100 = millis();
		/*
		 * So something every 100 ms
		 */
	}
}
