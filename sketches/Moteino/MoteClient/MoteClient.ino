// AVR Libraries
// #include <avsr/power.h>
// #include <avr/sleep.h>

// Arduino libraries
// #include <SoftwareSerial.h>

// Custom Arduino libraries
// #include "RTClib.h"     // https://github.com/adafruit/RTClib
// #include "IRremote.h"   // https://github.com/z3t0/Arduino-IRremote
// #include <Arduino_FreeRTOS.h>
// #include "semphr.h"  // add the FreeRTOS functions for Semaphores (or Flags).
// #include <MemoryFree.h>
// #include <Sleep_n0m1.h>
#include <dht.h>
#include "LowPower.h"
#include "PeriodicManager.h"

#define WITH_RFM69
#define WITH_SPIFLASH


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

#define 	USE_OLED

#ifdef USE_OLED
	#define I2C_ADDRESS 0x3C
	#include "SSD1306Ascii.h"
	#include "SSD1306AsciiAvrI2c.h"

	SSD1306AsciiAvrI2c oled;
#endif


// Custom defines
#define SERIAL_BR 	115200
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
dht DHT;
PeriodicManager pm;
// Sleep sleep;

void aboutToStart();
void updateOledData();

#define FREQ_1000_MS  1000
#define FREQ_500_MS   500
#define FREQ_100_MS   100

unsigned long ticks_1000 = 0;
unsigned long ticks_500  = 0;
unsigned long ticks_100  = 0;

volatile bool button_pushed;

void RSI_Red() {
  button_pushed = true;
}

void blinkLed()
{
	// static bool led_state = true;

	// digitalWrite(INFO_LED, led_state);
	// led_state = !led_state;

	Serial.print("t1: ");
    Serial.println(millis());
}

void setup() {

	// initialize serial communication
	Serial.begin(SERIAL_BR);
	while(!Serial);

	button_pushed = false;

	pm.addPeriodicTask(blinkLed);

	for(int p=0; p<PIN_COUNT; p++) {
		pinMode(p, INPUT);
		digitalWrite(p, LOW);
	}

	pinMode(INT_RED, INPUT_PULLUP);
	// pinMode(INFO_LED, OUTPUT);
	// pinMode(OLED_ENABLE, OUTPUT);



	ticks_1000 = millis();
	// ticks_500  = millis();
	// ticks_100  = millis();

#ifdef WITH_RFM69
	radio.initialize(FREQUENCY,NODEID,NETWORKID);
	radio.sleep();
#endif

#ifdef WITH_SPIFLASH
  if (flash.initialize())
    flash.sleep();
#endif

  aboutToStart();
}

long ticks = 0;

void loop()
{
	char buff[32];

	// ticks = millis();

	// if((ticks - ticks_1000) >= FREQ_1000_MS) {
	if(button_pushed == true) {
		delay(150);
		button_pushed = false;
		ticks_1000 = ticks;
		/*
		 * So something every 1000 ms
		 */

		pm.run();

		if(DHT.read22(DHT_PIN) == DHTLIB_OK)
			updateOledData();

    radio.send(GATEWAYID, "START", 6, false);

    // sleep.pwrDownMode(); //set sleep mode
    // sleep.sleepDelay(50000); //sleep for: sleepTime
	}

	// if((ticks - ticks_500) >= FREQ_500_MS) {
	// 	ticks_500 = ticks;
	// 	/*
	// 	 * So something every 500 ms
	// 	 */
	// 	// oled.setRow(2);
	// 	// oled.println(millis());
	// }

	// if((ticks - ticks_100) >= FREQ_100_MS) {
	// 	ticks_100 = ticks;
	// 	/*
	// 	 * So something every 100 ms
	// 	 */
	// 	// oled.setRow(3);
	// 	// oled.println(millis());
	// }

	// delay(50);

  // attachInterrupt(digitalPinToInterrupt(INT_RED), RSI_Red, FALLING);
  //
  // // Enter power down state with ADC and BOD module disabled.
  // // Wake up when wake up pin is low.
  // LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  //
  // // Disable external pin interrupt on wake up pin.
  // detachInterrupt(0);

  // Serial.end();
  radio.sleep();
  attachInterrupt(digitalPinToInterrupt(INT_RED), RSI_Red, FALLING);

	// set_sleep_mode( SLEEP_MODE_PWR_DOWN );
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
	// sleep_enable();
	// sleep_mode();

  detachInterrupt(digitalPinToInterrupt(INT_RED));

	// LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
	// sleep.pwrDownMode(); //set sleep mode
	// sleep.sleepDelay(128); //sleep for: sleepTime
	// ticks += 50;
	// ZZzzZZzzZZzz
}

void aboutToStart()
{
	for(int i=0; i<6; i++) {
		digitalWrite(INFO_LED, HIGH);
		delay(50);
		digitalWrite(INFO_LED, LOW);
		delay(50);
	}
}

void updateOledData()
{
#ifdef USE_OLED
	char buff[16];

	digitalWrite(OLED_ENABLE, HIGH);
	delay(200);
	oled.begin(&Adafruit128x64, I2C_ADDRESS);
	oled.setFont(Stang5x7);
	oled.clear();

	oled.home();
	oled.set2X();

	// sprintf(buff, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
	// oled.println(buff);

	sprintf(buff, "%d.%d C", int(DHT.temperature*10)/10, int(DHT.temperature*10)%10);
	oled.println(buff);

	sprintf(buff, "%d.%d Hum.", int(DHT.humidity*10)/10, int(DHT.humidity*10)%10);
	oled.println(buff);

	oled.println(millis());

	delay(2000);

	// LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
	digitalWrite(OLED_ENABLE, LOW);
#endif
}
