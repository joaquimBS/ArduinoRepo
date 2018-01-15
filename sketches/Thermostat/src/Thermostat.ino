// AVR Libraries
// #include <avr/power.h>
// #include <avr/sleep.h>

// Arduino libraries
// #include <SoftwareSerial.h>

// Custom Arduino libraries
#include "Thermostat.h"
// #include "RTClib.h"         // https://github.com/adafruit/RTClib
// #include "IRremote.h"       // https://github.com/z3t0/Arduino-IRremote
// #include <Arduino_FreeRTOS.h>
// #include "semphr.h"  // add the FreeRTOS functions for Semaphores (or Flags).
// #include <MemoryFree.h>
#include "dht.h"
#include "LowPower.h"           // https://github.com/rocketscream/Low-Power
// #include "FlashSpiFifo.h"

#include <SPI.h>           //comes with Arduino IDE (www.arduino.cc)
#include <RFM69.h>         //get it here: https://github.com/lowpowerlab/RFM69
#include <RFM69_ATC.h>     //get it here: https://github.com/lowpowerlab/RFM69
#include <RFM69_OTA.h>     //get it here: https://github.com/lowpowerlab/RFM69

#define ENABLE_VBAT_DIVISOR	 ; //digitalWrite(EN_VBAT_DIV, HIGH); delay(1000)
#define DISABLE_VBAT_DIVISOR ; // digitalWrite(EN_VBAT_DIV, LOW)

#define MAGIC_VBAT_OFFSET_MV	-40

#define WITH_RFM69
#define WITH_SPIFLASH
#define WITH_OLED

#ifdef WITH_OLED
#define I2C_ADDRESS 0x3C
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
SSD1306AsciiAvrI2c oled;
#define ENABLE_OLED_VCC		digitalWrite(OLED_VCC, HIGH)
#define DISABLE_OLED_VCC	digitalWrite(OLED_VCC, LOW)
#endif

#if defined(WITH_RFM69)
    RFM69 radio;
    #define GATEWAYID   1
    #define NETWORKID 100
    #define NODEID 11
    #define FREQUENCY RF69_868MHZ
    #define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#endif

#if defined(WITH_SPIFLASH)
    //*****************************************************************************************************************************
    // flash(SPI_CS, MANUFACTURER_ID)
    // SPI_CS          - CS pin attached to SPI flash chip (8 in case of Moteino)
    // MANUFACTURER_ID - OPTIONAL, 0x1F44 for adesto(ex atmel) 4mbit flash
    //                             0xEF30 for windbond 4mbit flash
    //                             0xEF40 for windbond 16/64mbit flash
    //*****************************************************************************************************************************
    SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for windbond 4mbit flash
#endif

typedef enum {
    C10=0,
    C15,
    C17,
    C20,
    C22,
    C_NUM
} t_tempratures;

const int fixed_temps[C_NUM] = {10, 15, 17, 20, 22};
int cur_fixed_temp = C15;
int desired_temp = fixed_temps[cur_fixed_temp];

typedef struct {
    int temp;
    int humi;
    int vbat_mv;
    int cycle_ms;
} t_sample_data;

t_sample_data last_sample = {0,0,0};

typedef enum {STATE_ON = 0, STATE_POWER_SAVE} t_system_state;
volatile t_system_state system_state;

typedef enum {CYCLIC = 0, INT_EXT} t_wake_up_cause;
volatile t_wake_up_cause wake_up_cause = INT_EXT;

// Custom defines
#define SERIAL_BR 115200
#define DHTTYPE   DHT22   // DHT 22  (AM2302), AM2321
dht DHT;

// FlashSpiFifo ffifo;

/* Function prototypes */
void periodicTask();
void RSI_Red();
void goToSleep();
void wakeUp();
/***********************/


#define CYCLES_OF_SLEEP_S   (unsigned int) 20
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

static int rele_value = LOW;

// ========================== End of Header ====================================

void RSI_Red()
{
    wake_up_cause = INT_EXT;
}

void init_io_pins()
{
    SET_DIGITAL_PINS_AS_INPUTS();

    // -- Custom IO setup --
    pinMode(OLED_VCC, OUTPUT);
    pinMode(BUTTON_IN, INPUT_PULLUP);
    pinMode(INFO_LED, OUTPUT);
    pinMode(RELAY_TRIGGER, INPUT_PULLUP);
    pinMode(RELAY_TRIGGER, INPUT);
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

    LED_OFF;

    sampleData();
    updateOled();
}


void button_pressed_callback()
{
    cur_fixed_temp = (++cur_fixed_temp%C_NUM);
    desired_temp = fixed_temps[cur_fixed_temp];

    DEBUG("cur_fixed_temp: "); DEBUGLN(cur_fixed_temp);
    DEBUG("desired_temp: "); DEBUGLN(desired_temp);

    updateOled();

    timer_to_sleep = millis() + TIMEOUT_TO_SLEEP_MS;
}

static uint16_t get_vbat_mv()
{
    analogReference(INTERNAL);	// Referencia interna de 1.1V

    uint16_t adc_vbat = analogRead(A7);

    for(int i=0; i<10; i++) {
        adc_vbat = analogRead(A7);
        delay(1);
    }

    float vbat = map(adc_vbat, 0, 1023, 0, 1100);	// Passem de la lectura 0-1023 de ADC a mV de 0-1100mV
    vbat *= 11;		// 11 és el factor de divisió del divisor.
    vbat = vbat + MAGIC_VBAT_OFFSET_MV;

    return (uint16_t)vbat;
}

void DoRelayPulse()
{
    pinMode(RELAY_TRIGGER, OUTPUT);
    delay(50);

    digitalWrite(RELAY_TRIGGER, LOW);
    delay(250);
    digitalWrite(RELAY_TRIGGER, HIGH);

    pinMode(RELAY_TRIGGER, INPUT_PULLUP);
}

bool isHeaterOn = false;

void HeaterOn()
{
    DEBUGLN("HeaterOn");
    if(isHeaterOn == true)
        return;

    DoRelayPulse();
    isHeaterOn = true;
}

void HeaterOff()
{
    DEBUGLN("HeaterOff");
    if(isHeaterOn == false)
        return;

    DoRelayPulse();
    isHeaterOn = false;
}

void thermostatLogic()
{
    if(last_sample.temp < desired_temp*10) {
        HeaterOn();
    }
    else if(last_sample.temp >= ((desired_temp+1)*10)){
        HeaterOff();
    }
}

void loop()
{
    // if (radio.receiveDone()) {
    // 	CheckForWirelessHEX(radio, flash, false);
    // }

    if(wake_up_cause == INT_EXT) {
        read_and_debounce_pushbutton();

        if(timer_to_sleep == TIMER_HOLD) {
            timer_to_sleep = millis() + TIMEOUT_TO_SLEEP_MS;
        }

        if(millis() > timer_to_sleep) {
            thermostatLogic();

            delay(1000);

            timer_to_sleep = TIMER_HOLD;
            remaining_sleep_cycles = CYCLES_OF_SLEEP_S;
            goToSleep();
        }
    }
    else {
        if(remaining_sleep_cycles == 0) {
            periodicSleepTask();
            remaining_sleep_cycles = CYCLES_OF_SLEEP_S;
        }
        else {
            remaining_sleep_cycles--;
            goToSleep();
        }
    }
}

void updateOled()
{
    char buff[16];

    oled.clear();
    oled.set2X();

    String str_temp_set = String((float)fixed_temps[cur_fixed_temp],1);
    String str_temp = String((last_sample.temp/10.0),1);
    String str_humi = String((last_sample.humi/10.0),1);
    String str_vbat = String((last_sample.vbat_mv/1000.0),2);

    sprintf(buff, "Set:  %sC", str_temp_set.c_str());
    oled.println(buff);

    sprintf(buff, "Real: %sC", str_temp.c_str(), str_humi.c_str());
    oled.println(buff);

    oled.set1X();
    sprintf(buff, "VBat: %s V", str_vbat.c_str());
    oled.println(buff);
}

#define TX_BUFF_LEN	((uint8_t) 10)
void txToBase()
{
    uint8_t tx_buff[TX_BUFF_LEN];

    tx_buff[0] = last_sample.temp & 0x00FF;
    tx_buff[1] = (last_sample.temp >> 8);

    tx_buff[2] = last_sample.humi & 0x00FF;
    tx_buff[3] = (last_sample.humi >> 8);

    tx_buff[4] = desired_temp & 0x00FF;
    tx_buff[5] = (desired_temp >> 8);

    tx_buff[6] = last_sample.vbat_mv & 0x00FF;
    tx_buff[7] = (last_sample.vbat_mv >> 8);

    tx_buff[8] = last_sample.cycle_ms; // Ha de ser menor que 255, sinó overflow

    tx_buff[9] = isHeaterOn;

    // radio.sendWithRetry(GATEWAYID, tx_buff, TX_BUFF_LEN, 2, 40);
    radio.send(GATEWAYID, tx_buff, TX_BUFF_LEN);
}

void sampleData()
{
    char buff[16];
    int safeguard_loop = 40;

    while((DHT.read22(DHT_PIN) != DHTLIB_OK) && (safeguard_loop-- > 0))
        delay(25);

    last_sample.temp = DHT.temperature * 10;
    last_sample.humi = DHT.humidity * 10;
    last_sample.vbat_mv = get_vbat_mv();

    sprintf(buff, "%d:%d:%d", last_sample.temp, last_sample.humi, last_sample.vbat_mv);
    DEBUGLN(buff);
}

void periodicAwakeTask()
{
    uint32_t t = millis();

    LED_ON;

    sampleData();
    updateOled();

    last_sample.cycle_ms = millis()-t;

    LED_OFF;
}

void periodicSleepTask()
{
    uint32_t t = millis();

    LED_ON;

    sampleData();
    txToBase();
    thermostatLogic();

    last_sample.cycle_ms = millis()-t;

    LED_OFF;
}

void goToSleep()
{
    // flash.sleep();   /* Only if it was awake. */
    radio.sleep();
    wake_up_cause = CYCLIC;

    DISABLE_OLED_VCC;
    pinMode(OLED_VCC, OUTPUT);
    pinMode(BUTTON_IN, INPUT_PULLUP);
    pinMode(INFO_LED, OUTPUT);
    pinMode(RELAY_TRIGGER, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(BUTTON_IN), RSI_Red, LOW);
    // Enter power down state with ADC and BOD module disabled.
    // Wake up when wake up pin is low.
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
//     delay(1000);
    // ZZzzZZzzZZzz

    // Disable external pin interrupt on wake up pin.
    detachInterrupt(digitalPinToInterrupt(BUTTON_IN));

    wakeUp();
}

/* Wake up routine */
void wakeUp()
{
    /* Do some wake up routines. */
    /* radio doesn't need to be wake up. */
    if(wake_up_cause == INT_EXT) {
        init_oled();
        updateOled();
    }
}

#define DEBOUNCE_TIME_MS    100
#define PB_PRESSED          LOW
#define PB_RELEASED         HIGH

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
            if(digitalRead(BUTTON_IN) == PB_PRESSED) {
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
            else if(digitalRead(BUTTON_IN) == PB_RELEASED) {
                pb_state = PB_IDLE;
            }
            else {
                /* Keep the current state */
            }
            break;

        case PB_PUSH_CONFIRMED:
            if(digitalRead(BUTTON_IN) == PB_RELEASED) {
                button_pressed_callback();
                pb_state = PB_IDLE;
            }
            else {
                /* Keep the current state */
            }
            break;
    }
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
#ifdef WITH_OLED
    ENABLE_OLED_VCC;
    delay(500);

    oled.begin(&Adafruit128x64, I2C_ADDRESS);
    oled.setFont(Stang5x7);
    oled.clear();

    oled.home();
    oled.set2X();

    oled.println("Thermostat ");
    oled.println("v1         ");
#endif
}
