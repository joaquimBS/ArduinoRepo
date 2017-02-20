// AVR Libraries
// #include <avr/power.h>
// #include <avr/sleep.h>

// Arduino libraries
// #include <SoftwareSerial.h>

// Custom Arduino libraries
#include "../lib/project_cfg.h"
// #include "RTClib.h"         // https://github.com/adafruit/RTClib
// #include "IRremote.h"       // https://github.com/z3t0/Arduino-IRremote
// #include <Arduino_FreeRTOS.h>
// #include "semphr.h"  // add the FreeRTOS functions for Semaphores (or Flags).
// #include <MemoryFree.h>
#include "dht.h"
#include "LowPower.h"           // https://github.com/rocketscream/Low-Power
// #include <Sleep_n0m1.h>
#include "FlashSpiFifo.h"

#define ENABLE_MIC	digitalWrite(MIC_VCC, HIGH); delay(100)
#define DISABLE_MIC	digitalWrite(MIC_VCC, LOW)

#define WITH_RFM69
#define WITH_SPIFLASH

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
		#include "FlashSpiFifo.h"
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

FlashSpiFifo ffifo;

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

	pinMode(INFO_LED, OUTPUT);
	pinMode(MIC_VCC, OUTPUT);

	LED_ON;

	#ifdef WITH_RFM69
	radio.initialize(FREQUENCY,NODEID,NETWORKID);
	radio.setHighPower();
	radio.encrypt(ENCRYPTKEY);
	radio.sleep();
	#endif

	ffifo.init();

	char buff[50];
	sprintf(buff, "Transmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
	DEBUGLN(buff);

	LED_OFF;
}

#define CYCLES_OF_SLEEP_S   (unsigned int) 10
#define TIMER_HOLD          (unsigned int) -1
#define TIMEOUT_TO_SLEEP_MS (unsigned int) 10000
#define SLEEP_CYC_10S		(unsigned int) 10
#define SLEEP_CYC_5S		(unsigned int) 5
#define SLEEP_CYC_2S		(unsigned int) 2
#define SLEEP_CYC_1S		(unsigned int) 1

int read_time;
int cycles = 0;
long timer_to_sleep = TIMER_HOLD;
uint8_t remaining_sleep_cycles = 0;

void loop()
{
	if(wake_up_cause == INT_EXT) {
		// if(timer_to_sleep == TIMER_HOLD) {
		// 	timer_to_sleep = millis();
		// }
		//
		// if((millis() - timer_to_sleep) >= TIMEOUT_TO_SLEEP_MS) {
		// 	timer_to_sleep = TIMER_HOLD;
		// 	remaining_sleep_cycles = SLEEP_CYC_5S;
		// 	goToSleep();
		// }
	}
	else {
		if(remaining_sleep_cycles == 0) {
			periodicTask();
			remaining_sleep_cycles = 5;
		}
		else {
			remaining_sleep_cycles--;
			goToSleep();
		}
	}
}

#define TX_BUFF_LEN	((uint8_t) 6)
void periodicTask()
{
	// uint32_t t=millis();

	uint8_t tx_buff[TX_BUFF_LEN];

	// t_fifo_data fd;
	// fd.temperature = 22;
	// ffifo.push(fd);

	int16_t temp_tx = 0;
	int16_t humi_tx = 0;
	uint16_t noise_tx = 0;

	LED_ON;

	if(DHT.read22(DHT_PIN) == DHTLIB_OK) {
		temp_tx = DHT.temperature * 10;
		humi_tx = DHT.humidity * 10;
	}

	ENABLE_MIC;
	noise_tx = analogRead(A0);
	DISABLE_MIC;

	tx_buff[0] = temp_tx & 0x00FF;
	tx_buff[1] = (temp_tx >> 8);

	tx_buff[2] = humi_tx & 0x00FF;
	tx_buff[3] = (humi_tx >> 8);

	tx_buff[4] = noise_tx & 0x00FF;
	tx_buff[5] = (noise_tx >> 8);

	// radio.sendWithRetry(GATEWAYID, tx_buff, TX_BUFF_LEN, 2, 40);
	radio.send(GATEWAYID, tx_buff, TX_BUFF_LEN);

	LED_OFF;

	// DEBUGLN(DHT.temperature);
	// DEBUGLN(DHT.humidity);
	// if(noise_tx < 500) {
		// DEBUGLN(noise_tx);
		// delay(500);
	// }

	// Serial.println(millis()-t);
}

void goToSleep()
{
	// flash.sleep();   /* Only if it was awake. */
	radio.sleep();
	wake_up_cause = CYCLIC;

	// attachInterrupt(digitalPinToInterrupt(INT_RED), RSI_Red, LOW);
	// Enter power down state with ADC and BOD module disabled.
	// Wake up when wake up pin is low.
	LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
	// delay(20);
	// ZZzzZZzzZZzz

	// Disable external pin interrupt on wake up pin.
	// detachInterrupt(digitalPinToInterrupt(INT_RED));

	wakeUp();
}

/* Wake up routine */
void wakeUp()
{
	/* Do some wake up routines. */
	/* radio doesn't need to be wake up. */
}
