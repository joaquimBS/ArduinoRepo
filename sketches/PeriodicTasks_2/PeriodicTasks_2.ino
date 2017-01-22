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
#include <dht.h>
#include <TaskScheduler.h>
// #include <MemoryFree.h>

// I/O Pins
enum {P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, INFO_LED, PIN_COUNT};

// Custom defines
#define SERIAL_BAUDRATE 	115200

// Callback methods prototypes
void t1Callback();

//Tasks
Task t1(5000, TASK_FOREVER, &t1Callback);

Scheduler runner;

void t1Callback() {
	static bool led_status = HIGH;
    // Serial.print("t1: ");
    // Serial.println(millis());

    digitalWrite(INFO_LED, led_status);
    led_status = !led_status;
}

void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(SERIAL_BAUDRATE);
  while(!Serial);

  for(int p=0; p<PIN_COUNT; p++) {
  	pinMode(p, INPUT);
  }

  pinMode(INFO_LED, OUTPUT);

  runner.init();
  // runner.allowSleep(true);
  Serial.println("Initialized scheduler");
  
  runner.addTask(t1);
  Serial.println("added t1");
  
  // runner.addTask(t2);
  // Serial.println("added t2");

  // delay(5000);
  
  t1.enable();
  Serial.println("Enabled t1");
}

void loop()
{
	runner.execute();
}
