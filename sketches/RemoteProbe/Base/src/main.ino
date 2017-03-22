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
// #include "dht.h"
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
#define ENABLE_OLED_VCC		digitalWrite(OLED_VCC, HIGH)
#define DISABLE_OLED_VCC	digitalWrite(OLED_VCC, LOW)
#endif

#if defined(WITH_RFM69) || defined(WITH_SPIFLASH)
#include <SPI.h>                //comes with Arduino IDE (www.arduino.cc)
#if defined(WITH_RFM69)
#include <RFM69.h>            //get it here: https://www.github.com/lowpowerlab/rfm69
RFM69 radio;
	#define GATEWAYID	1
	#define NETWORKID	100
	#define NODEID 		1
	#define FREQUENCY	RF69_868MHZ
	#define ENCRYPTKEY	"sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#endif
#if defined(WITH_SPIFLASH)
#include <SPIFlash.h>         //get it here: https://www.github.com/lowpowerlab/spiflash
SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)
#endif
#endif

typedef enum {
	PS_POWER_SAVE = 0,
	PS_ON
} t_power_state;

typedef enum {
	SS_MAIN_IDLE = 0
} t_system_state;

typedef enum {
	CYCLIC = 0,
	INT_EXT
} t_wake_up_cause;

volatile t_power_state power_state;
volatile t_system_state system_state;
volatile t_wake_up_cause wake_up_cause;

/* Function prototypes */
void periodic_task();
void init_io_pins();
void init_radio();
void init_flash();
void init_oled();
void power_state_on_entry();
void power_state_power_save_entry();
void go_to_sleep();
void wake_up_from_sleep();
/***********************/
// ========================== End of Header ====================================

/**
 * This function should never be called. It is reserved to RFM69W module
 */
static void rsi_int_0()
{
	/* Do nothing */
}

/**
 * This function will be called when the hardware INT1 is triggered.
 */
static void rsi_int_1()
{
	power_state = PS_ON;
	system_state = SS_MAIN_IDLE;
}

void setup()
{
	Serial.begin(SERIAL_BR);
	while(!Serial);

	init_io_pins();

	LED_ON;

	init_radio();
	init_flash();
	init_oled();

	power_state = PS_ON;
	system_state = SS_MAIN_IDLE;

	char buff[50];
	sprintf(buff, "Receiving at %d MHz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
	Serial.println(buff);

	LED_OFF;
}

void loop()
{
	switch (power_state) {
		case PS_ON:
			LED_ON;
			power_state_on_entry();
			break;

		case PS_POWER_SAVE:
			LED_OFF;
			power_state_power_save_entry();
			break;

		default:
			delay(100);
			break;
	}
}

void button_pressed_callback()
{
	power_state = PS_POWER_SAVE;
	DEBUGLN("button_pressed_callback");
}

typedef enum {
	PB_IDLE = 0,
	PB_DEBOUCE,
	PB_PUSH_CONFIRMED
} t_push_button_state;

/*
 * This function needs to be called very often.
 * It implements a FSM IDLE -> DEBOUNCE -> CONFIRM to read a switch value
 */
static void read_and_debounce_pushbutton()
{
	static t_push_button_state pb_state = PB_IDLE;
	static unsigned long tick_time = 0;

	switch (pb_state) {
		case PB_IDLE:
			if(digitalRead(INT_RED) == PB_PRESSED) {
				pb_state = PB_DEBOUCE;
				tick_time = millis();
			}
			else {
				/* Keep the current state */
			}
			break;

		case PB_DEBOUCE:
			if((millis() - tick_time) > DEBOUNCE_TIME_MS) {
				pb_state = PB_PUSH_CONFIRMED;
			}
			else if(digitalRead(INT_RED) == PB_RELEASED) {
				pb_state = PB_IDLE;
			}
			else {
				/* Keep the current state */
			}
			break;

		case PB_PUSH_CONFIRMED:
			if(digitalRead(INT_RED) == PB_RELEASED) {
				button_pressed_callback();
				pb_state = PB_IDLE;
			}
			else {
				/* Keep the current state */
			}
			break;
	}
}

void power_state_power_save_entry()
{
	go_to_sleep();
}

void power_state_on_entry()
{
	/*
	switch (system_state) {
		case SS_MAIN_IDLE:
			periodic_task();

	}
	*/

	read_and_debounce_pushbutton();

	if (radio.receiveDone()) {
		LED_ON;
		update_oled_view();
		LED_OFF;
	}
}

void update_oled_view()
{
#ifdef USE_OLED
	char buff[16];

	oled.clear();

	int16_t rx_temp = 0;
	int16_t rx_humi = 0;
	uint16_t rx_micr = 0;
	uint16_t rx_vbat = 0;
	uint8_t rx_sample_time = 0;

	rx_temp = radio.DATA[0] + (radio.DATA[1] << 8);
	rx_humi = radio.DATA[2] + (radio.DATA[3] << 8);
	rx_micr = radio.DATA[4] + (radio.DATA[5] << 8);
	rx_vbat = radio.DATA[6] + (radio.DATA[7] << 8);
	rx_sample_time = radio.DATA[8];

	oled.set2X();
	sprintf(buff, "%d.%dC  %d%%", rx_temp/10, rx_temp%10, rx_humi/10);
	oled.println(buff);

	oled.set1X();
	sprintf(buff, "%d Mic.", rx_micr);
	oled.println(buff);
	sprintf(buff, "t=%d ms", rx_sample_time);
	oled.println(buff);
	sprintf(buff, "RSSI:%d", radio.RSSI);
	oled.println(buff);
	sprintf(buff, "VBat: %d.%d V", rx_vbat/1000, (rx_vbat%1000)/10);
	oled.println(buff);

	sprintf(buff, "%d;%d;%d", rx_temp, rx_humi, rx_micr);
	DEBUGLN(buff);

	// /* Keep count of the Rx frames */
	// static long i=0;
	// oled.println(i++);
#endif
}

void go_to_sleep()
{
	// flash.sleep();   /* Only if it was awake. */
	radio.sleep();
	DISABLE_OLED_VCC;

	pinMode(OLED_VCC, INPUT);
	pinMode(INFO_LED, INPUT);

	delay(500);		// To make sure no bouncing when INT attached.

	attachInterrupt(digitalPinToInterrupt(INT_RED), rsi_int_1, LOW);
	/*********************************/
	while(power_state == PS_POWER_SAVE)
		LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
		// delay(1000);
	// ZZzzZZzzZZzz....
	/*********************************/
	detachInterrupt(digitalPinToInterrupt(INT_RED));

	while(digitalRead(INT_RED) == LOW);

	// Here we are awake and the pushbutton is un-pressed
	wake_up_from_sleep();
}

/* Wake up routine */
void wake_up_from_sleep()
{
	pinMode(OLED_VCC, OUTPUT);
	pinMode(INFO_LED, OUTPUT);

	init_oled();
}

void init_io_pins()
{
	SET_DIGITAL_PINS_AS_INPUTS();

	// -- Custom IO setup --
	pinMode(OLED_VCC, OUTPUT);
	pinMode(INT_RED, INPUT_PULLUP);
	pinMode(INFO_LED, OUTPUT);
	pinMode(A0, INPUT);
}

void init_radio()
{
#ifdef WITH_RFM69
	radio.initialize(FREQUENCY,NODEID,NETWORKID);
	radio.setHighPower();
	radio.encrypt(ENCRYPTKEY);
	radio.sleep();
#endif
}

void init_flash()
{
#ifdef WITH_SPIFLASH
	if (flash.initialize()) {
		flash.sleep();
	}
#endif
}

void init_oled()
{
#ifdef USE_OLED
	ENABLE_OLED_VCC;
	delay(500);

	oled.begin(&Adafruit128x64, I2C_ADDRESS);
	oled.setFont(Stang5x7);
	oled.clear();

	oled.home();
	oled.set2X();

	oled.println("           ");
	oled.println("   HOLA!   ");
#endif
}
