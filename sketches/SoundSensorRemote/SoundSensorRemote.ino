#include "SoundSensorRemote.h"

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

    digitalWrite(INFO_LED, HIGH);

#ifdef WITH_RFM69
    radio.initialize(FREQUENCY,NODEID,NETWORKID);
    radio.sleep();
#endif

#ifdef WITH_SPIFLASH
    if (flash.initialize()) {
        flash.sleep();
    }
#endif

    digitalWrite(INFO_LED, LOW);
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
            
        }

        if(DHT.read22(DHT_PIN) == DHTLIB_OK) {
            /* Do something with the data. */
        }

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
}

/* Wake up routine */
void wakeUp()
{
    /* Do some wake up routines. */
}
