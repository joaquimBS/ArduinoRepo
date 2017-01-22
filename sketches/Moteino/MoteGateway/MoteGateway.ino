#include "MoteGateway.h"

void goToSleep()
{
    digitalWrite(OLED_ENABLE, LOW);
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
}

void RSI_Red() {
    /* Do nothing */
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
    // pinMode(OLED_ENABLE, OUTPUT);

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

#define CYCLES_OF_SLEEP_S   (unsigned int) 30
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

            /* Wake up routine */
            InitOled();
        }

        if(DHT.read22(DHT_PIN) == DHTLIB_OK)
            updateOledData();

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
    DHT.read22(DHT_PIN);
}

void aboutToStart()
{
    for(int i=0; i<6; i++) {
        digitalWrite(INFO_LED, HIGH); delay(50);
        digitalWrite(INFO_LED, LOW);  delay(50);
    }
}

void InitOled()
{
#ifdef WITH_OLED
	digitalWrite(OLED_ENABLE, HIGH);
	delay(150);
	oled.begin(&Adafruit128x64, I2C_ADDRESS);
	oled.setFont(Stang5x7);
    oled.set2X();
#endif
}

void updateOledData()
{
#ifdef WITH_OLED
	char buff[16];

	// oled.clear();
    oled.home();

	// sprintf(buff, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
	// oled.println(buff);

	sprintf(buff, "%d.%d C", int(DHT.temperature*10)/10, int(DHT.temperature*10)%10);
	oled.println(buff);

	sprintf(buff, "%d.%d Hum.", int(DHT.humidity*10)/10, int(DHT.humidity*10)%10);
	oled.println(buff);

	oled.println(millis());
    oled.println(read_time);

	// delay(1000);
#endif
}
