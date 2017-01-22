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
#include "dht.h"
#include "LowPower.h"           // https://github.com/rocketscream/Low-Power
// #include <Sleep_n0m1.h>

#define WITH_RFM69
#define WITH_SPIFLASH

// I/O Pins
enum {P0, P1, P2, INT_RED, P4, P5, DHT_PIN, P7, FLASH_SS, INFO_LED, P10, P11, P12, P13, PIN_COUNT};	/* Moteino */

#define LED_ON      digitalWrite(INFO_LED, HIGH)
#define LED_OFF     digitalWrite(INFO_LED, LOW)

#if defined(WITH_RFM69) || defined(WITH_SPIFLASH)
#include <SPI.h>                //comes with Arduino IDE (www.arduino.cc)
#if defined(WITH_RFM69)
#include <RFM69.h>            //get it here: https://www.github.com/lowpowerlab/rfm69
RFM69 radio;
#define GATEWAYID   1
#define NETWORKID 100
#define NODEID 11
#define FREQUENCY RF69_868MHZ
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#endif
#if defined(WITH_SPIFLASH)
#include <SPIFlash.h>         //get it here: https://www.github.com/lowpowerlab/spiflash
SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)
#endif
#endif

typedef enum {STATE_ON = 0, STATE_POWER_SAVE} t_system_state;
volatile t_system_state system_state;

typedef enum {CYCLIC = 0, INT_EXT} t_wake_up_cause;
volatile t_wake_up_cause wake_up_cause;

// Custom defines
#define SERIAL_BR 115200
#define DHTTYPE   DHT22   // DHT 22  (AM2302), AM2321
dht DHT;

/* Function prototypes */
void periodicTask();
void RSI_Red();
void goToSleep();
void wakeUp();
void updateOledData();
/***********************/

// ========================== End of Header ====================================

void RSI_Red()
{
	wake_up_cause = INT_EXT;
}

void setup()
{
	Serial.begin(SERIAL_BR);
	while(!Serial);

	for(int p=0; p<PIN_COUNT; p++) {
		pinMode(p, INPUT);
		digitalWrite(p, LOW);
	}

	pinMode(INT_RED, INPUT_PULLUP);
	pinMode(INFO_LED, OUTPUT);

	LED_ON;

	#ifdef WITH_RFM69
	radio.initialize(FREQUENCY,NODEID,NETWORKID);
	radio.setHighPower();
	radio.encrypt(ENCRYPTKEY);
	radio.sleep();
	#endif

	#ifdef WITH_SPIFLASH
	if (flash.initialize()) {
		flash.sleep();
	}
	#endif

	char buff[50];
	sprintf(buff, "\Transmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
	Serial.println(buff);

	LED_OFF;
}

#define CYCLES_OF_SLEEP_S   (unsigned int) 10
#define TIMER_HOLD          (unsigned int) -1
#define TIMEOUT_TO_SLEEP_MS (unsigned int) 10000

static int read_time;
static int cycles = 0;
static long timer_to_sleep = TIMER_HOLD;

void loop()
{
	if(wake_up_cause == INT_EXT) {
		if(timer_to_sleep == TIMER_HOLD) {
			timer_to_sleep = millis();

		}

		// if(DHT.read22(DHT_PIN) == DHTLIB_OK) {
		//     /* Do something with the data. */
		// }

		if((millis() - timer_to_sleep) >= TIMEOUT_TO_SLEEP_MS) {
			timer_to_sleep = TIMER_HOLD;
			cycles = CYCLES_OF_SLEEP_S;
			goToSleep();
		}
	}
	else {
		if(cycles == 0) {
			periodicTask();
			cycles = CYCLES_OF_SLEEP_S;
		}
		else {
			cycles--;
			goToSleep();
		}
	}
}

void periodicTask()
{
	char buff[8];

	if(DHT.read22(DHT_PIN) == DHTLIB_OK) {
		LED_ON;
		sprintf(buff, "%d.%d C", int(DHT.temperature*10)/10, int(DHT.temperature*10)%10);

		// sprintf(buff, "%d.%d Hum.", int(DHT.humidity*10)/10, int(DHT.humidity*10)%10);
		// oled.println(buff);

		int sendSize = strlen(buff);
		radio.sendWithRetry(GATEWAYID, buff, sendSize, 2, 40);
		radio.send(GATEWAYID, buff, sendSize);
		LED_OFF;
	}

	remaining_sleep_cycles = SLEEP_CYC_10S;
}

void goToSleep()
{
	// flash.sleep();   /* Only if it was awake. */
	radio.sleep();
	wake_up_cause = CYCLIC;

	attachInterrupt(digitalPinToInterrupt(INT_RED), RSI_Red, LOW);
	// Enter power down state with ADC and BOD module disabled.
	// Wake up when wake up pin is low.
	LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
	// ZZzzZZzzZZzz

	// Disable external pin interrupt on wake up pin.
	detachInterrupt(digitalPinToInterrupt(INT_RED));

	wakeUp();
}

/* Wake up routine */
void wakeUp()
{
	/* Do some wake up routines. */
	/* radio doesn't need to be wake up. */

}
