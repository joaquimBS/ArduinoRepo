// AVR Libraries
// #include <avr/power.h>
// #include <avr/sleep.h>

// Arduino libraries
// #include <SoftwareSerial.h>

// Custom Arduino libraries
#include "Thermostat.h"
// #include "RTClib.h"         // https://github.com/adafruit/RTClib
// #include <MemoryFree.h>
#include "dht.h"
#include "LowPower.h"           // https://github.com/rocketscream/Low-Power
// #include "FlashSpiFifo.h"

#include <SPI.h>           //comes with Arduino IDE (www.arduino.cc)
#include <RFM69.h>         //get it here: https://github.com/lowpowerlab/RFM69
//#include <RFM69_ATC.h>     //get it here: https://github.com/lowpowerlab/RFM69
#include <RFM69_OTA.h>     //get it here: https://github.com/lowpowerlab/RFM69

#define ENABLE_VBAT_DIVISOR	 ; //digitalWrite(EN_VBAT_DIV, HIGH); delay(1000)
#define DISABLE_VBAT_DIVISOR ; // digitalWrite(EN_VBAT_DIV, LOW)

#define TX_BUFF_LEN	((uint8_t) 10)
#define MAGIC_VBAT_OFFSET_MV ((int8_t) -40)

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

// Custom defines
#define SERIAL_BR 115200
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define TIME_INCREMENT_S 10
#define TIME_MAX_S (4*3600)

void thermo_power_on_pre();
void thermo_power_on_during();
void thermo_power_save_pre();
void thermo_power_save_during();

dht DHT;

typedef enum {
    MODE_TIME=0,
    MODE_TEMP
} t_thermo_mode;

typedef enum {
    HEATER_OFF=0,
    HEATER_ON
} t_heater_status;

typedef enum {
    POWER_ON=0,
    POWER_SAVE
} t_thermo_power_mode;

typedef struct {
    t_heater_status heater_status;
    t_thermo_mode mode;
    t_thermo_power_mode power_mode;
    uint16_t remaining_time_s;
    uint16_t humidity;
    uint16_t temperature;
    uint16_t setpoint;
    uint16_t vbat_mv;
} t_thermo_data;

t_thermo_data td = {0};

typedef void (*t_callback)(void);

typedef struct {
    t_callback pre_callback;
    t_callback during_callback;
    t_callback post_callback;
} t_state_functions;

static t_state_functions state_power_on{thermo_power_on_pre, thermo_power_on_during};
static t_state_functions state_power_save{thermo_power_save_pre, thermo_power_save_during};
static t_state_functions *current_state = &state_power_on;


typedef enum {CYCLIC = 0, INT_EXT} t_wake_up_cause;
volatile t_wake_up_cause wake_up_cause = CYCLIC;

/* Function prototypes */
void periodicTask();
void RSI_Red();
void go_to_sleep();
void wakeUp();
/***********************/


#define CYCLES_OF_SLEEP_S   (unsigned int) 20
#define TIMER_HOLD          (unsigned int) -1
#define TIMEOUT_TO_SLEEP_MS (unsigned int) 10000
#define SLEEP_CYC_10S		(unsigned int) 10
#define SLEEP_CYC_5S		(unsigned int) 5
#define SLEEP_CYC_2S		(unsigned int) 2
#define SLEEP_CYC_1S		(unsigned int) 1

long timer_to_sleep = TIMER_HOLD;
uint8_t remaining_sleep_cycles = 0;
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

    set_thermo_state(&state_power_on);
}

void loop()
{
    // if (radio.receiveDone()) {
    // 	CheckForWirelessHEX(radio, flash, false);
    // }

    current_state->during_callback();
}

void button_pressed_callback()
{
    /* Some code */
    // current_view->red_button_pressed();

    switch (td.mode) {
    case MODE_TIME:
        td.remaining_time_s += TIME_INCREMENT_S;
        if(td.remaining_time_s > TIME_MAX_S) {
            td.remaining_time_s = 0;
        }
        break;
    case MODE_TEMP:

        break;

    default:
        break;
    }

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

bool is_heater_on = false;

void heater_on()
{
    if(td.heater_status == HEATER_ON)
        return;

    DoRelayPulse();
    td.heater_status = HEATER_ON;
    LED_ON;
}

void heater_off()
{
    if(td.heater_status == HEATER_OFF)
        return;

    DoRelayPulse();
    td.heater_status = HEATER_OFF;
    LED_OFF;
}

void thermo_logic()
{
    switch (td.mode) {
    case MODE_TIME:
        if(td.remaining_time_s > 0)
            heater_on();
        else
            heater_off();
        break;
    case MODE_TEMP:

        break;

    default:
        break;
    }
}

void thermo_power_on_pre()
{
    timer_to_sleep = millis() + TIMEOUT_TO_SLEEP_MS;

    sampleData();
    updateOled();
}

void thermo_power_on_during()
{
    static long long timer_1s = millis() + 1000;

    if(millis() > timer_1s) {
        timer_1s = millis() + 1000;

        if(td.remaining_time_s > 0)
            td.remaining_time_s--;

        updateOled();
    }

    read_and_debounce_pushbutton();

    if(timer_to_sleep == TIMER_HOLD) {
        timer_to_sleep = millis() + TIMEOUT_TO_SLEEP_MS;
    }

    if(millis() > timer_to_sleep) {
        thermo_logic();
        set_thermo_state(&state_power_save);
    }
}

void thermo_power_save_pre()
{
    timer_to_sleep = TIMER_HOLD;
    remaining_sleep_cycles = CYCLES_OF_SLEEP_S;
}

void thermo_power_save_during()
{
    if(remaining_sleep_cycles == 0) {
        periodicSleepTask();
        remaining_sleep_cycles = CYCLES_OF_SLEEP_S;
    }
    else {
        if(td.remaining_time_s > 0)
            td.remaining_time_s--;

        remaining_sleep_cycles--;
        go_to_sleep();
    }
}

void set_thermo_state(t_state_functions *new_state)
{
    current_state = new_state;
    current_state->pre_callback();
}

void updateOled()
{
    char buff[16];

    oled.clear();
    oled.set2X();

    oled.println(td.remaining_time_s);

//    String str_setpoint = String((td.setpoint/10.0),1);
//    String str_temp = String((td.temperature/10.0),1);
//    String str_humi = String((td.humidity/10.0),1);
//    String str_vbat = String((td.vbat_mv/1000.0),2);

//    sprintf(buff, "Set:  %sC", str_setpoint.c_str());
//    oled.println(buff);

//    sprintf(buff, "Real: %sC", str_temp.c_str(), str_humi.c_str());
//    oled.println(buff);

//    oled.set1X();
//    sprintf(buff, "VBat: %s V", str_vbat.c_str());
//    oled.println(buff);
}

void txToBase()
{
    uint8_t tx_buff[TX_BUFF_LEN];

    tx_buff[0] = td.temperature & 0x00FF;
    tx_buff[1] = (td.temperature >> 8);

    tx_buff[2] = td.humidity & 0x00FF;
    tx_buff[3] = (td.humidity >> 8);

    tx_buff[4] = td.setpoint & 0x00FF;
    tx_buff[5] = (td.setpoint >> 8);

    tx_buff[6] = td.vbat_mv & 0x00FF;
    tx_buff[7] = (td.vbat_mv >> 8);

//    tx_buff[8] = last_sample.cycle_ms; // Ha de ser menor que 255, sinó overflow

    tx_buff[9] = td.heater_status;

    // radio.sendWithRetry(GATEWAYID, tx_buff, TX_BUFF_LEN, 2, 40);
    radio.send(GATEWAYID, tx_buff, TX_BUFF_LEN);
}

void sampleData()
{
    char buff[16];
    int safeguard_loop = 40;

    while((DHT.read22(DHT_PIN) != DHTLIB_OK) && (safeguard_loop-- > 0))
        delay(25);

    td.temperature = DHT.temperature * 10;
    td.humidity = DHT.humidity * 10;
    td.vbat_mv = get_vbat_mv();

    sprintf(buff, "%d:%d:%d:%d", td.heater_status, td.temperature, td.humidity, td.vbat_mv);
    DEBUGLN(buff);
}

void periodicSleepTask()
{
    uint32_t t = millis();

//    LED_ON;

    sampleData();
    txToBase();
    thermo_logic();

//    last_sample.cycle_ms = millis()-t;

//    LED_OFF;
}

void go_to_sleep()
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
        set_thermo_state(&state_power_on);
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
