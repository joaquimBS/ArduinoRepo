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
// #define WITH_SPIFLASH
#define 	USE_OLED

#ifdef USE_OLED
#define I2C_ADDRESS 0x3C
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
SSD1306AsciiAvrI2c oled;
#endif

// I/O Pins
enum {P0, P1, P2, INT_RED, DHT_PIN, P5, P6, P7, FLASH_SS, INFO_LED, P10, P11, P12, P13, PIN_COUNT};	/* Moteino */

#define LED_ON      digitalWrite(INFO_LED, HIGH)
#define LED_OFF     digitalWrite(INFO_LED, LOW)

#if defined(WITH_RFM69) || defined(WITH_SPIFLASH)
#include <SPI.h>                //comes with Arduino IDE (www.arduino.cc)
#if defined(WITH_RFM69)
#include <RFM69.h>            //get it here: https://www.github.com/lowpowerlab/rfm69
RFM69 radio;
#define GATEWAYID   1
#define NETWORKID 100
#define NODEID 1
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

/* Function prototypes */
void periodicTask();
void InitializeRadio();
void InitializeFlash();
void InitializeOled();
/***********************/

// ========================== End of Header ====================================

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

	InitializeRadio();
	InitializeFlash();
	InitializeOled();

	char buff[50];
	sprintf(buff, "Receiving at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
	Serial.println(buff);

	LED_OFF;
}

#define CYCLES_OF_SLEEP_S   (unsigned int) 1
#define TIMER_HOLD          (unsigned int) -1
#define TIMEOUT_TO_SLEEP_MS (unsigned int) 10000

static int read_time;
static int cycles = 0;
static long timer_to_sleep = TIMER_HOLD;

void loop()
{
	if (radio.receiveDone()) {
		LED_ON;

		updateOledData();

		LED_OFF;
	}
}

void updateOledData()
{
	#ifdef USE_OLED
	char buff[16];

	oled.clear();

	// Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
	for (byte i = 0; i < radio.DATALEN; i++) {
		buff[i] = (char)radio.DATA[i];
	}

	oled.println(buff);

	sprintf(buff, "RSSI:%d", radio.RSSI);
	oled.println(buff);

	/* Keep count of the Rx frames */
	static long i=0;
	oled.println(i++);
	#endif
}

void InitializeRadio()
{
#ifdef WITH_RFM69
	radio.initialize(FREQUENCY,NODEID,NETWORKID);
	radio.setHighPower();
	radio.encrypt(ENCRYPTKEY);
	radio.sleep();
#endif
}

void InitializeFlash()
{
#ifdef WITH_SPIFLASH
	if (flash.initialize()) {
		flash.sleep();
	}
#endif
}

void InitializeOled()
{
#ifdef USE_OLED
	oled.begin(&Adafruit128x64, I2C_ADDRESS);
	oled.setFont(Stang5x7);
	oled.clear();

	oled.home();
	oled.set2X();
#endif
}
