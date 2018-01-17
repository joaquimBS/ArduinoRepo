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

#define ENABLE_VBAT_DIVISOR  ; //digitalWrite(EN_VBAT_DIV, HIGH); delay(1000)
#define DISABLE_VBAT_DIVISOR ; // digitalWrite(EN_VBAT_DIV, LOW)

#define TX_BUFF_LEN ((uint8_t) 10)
#define MAGIC_VBAT_OFFSET_MV ((int8_t) -40)

#define WITH_RFM69
#define WITH_SPIFLASH
#define WITH_OLED

#ifdef WITH_OLED
#define I2C_ADDRESS 0x3C
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
SSD1306AsciiAvrI2c oled;
#define ENABLE_OLED_VCC  digitalWrite(OLED_VCC, HIGH)
#define DISABLE_OLED_VCC digitalWrite(OLED_VCC, LOW)
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

dht DHT;

#define SHORT_CLICK_TIME_MS 100
#define LONG_CLICK_TIME_MS 1000
#define VERYLONG_CLICK_TIME_MS 3000
#define PB_PRESSED LOW
#define PB_RELEASED HIGH

// Custom defines
#define SERIAL_BR 115200
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

#define TIME_ZERO ((uint16_t)0)
#define TIMER_DISABLED ((uint16_t)-1)

#ifdef USE_DEBUG
#define TIMEOUT_TO_SLEEP_MS ((unsigned int)3000)
#define TIME_INCREMENT_S ((unsigned int)10)
#define TIME_TO_OFF_MAX_S ((unsigned int)90)
#define TIME_TO_ON_MAX_S ((unsigned int)90)
#define TIME_TO_ON_MAX_TIME_AFTER_S ((unsigned int)5)
#define CYCLES_OF_SLEEP_S ((unsigned int)20)
#else
#define TIMEOUT_TO_SLEEP_MS ((unsigned int)10000)
#define TIME_INCREMENT_S ((unsigned int)1800)
#define TIME_TO_OFF_MAX_S ((unsigned int)4*3600)
#define TIME_TO_ON_MAX_S ((unsigned int)12*3600)
#define TIME_TO_ON_MAX_TIME_AFTER_S ((unsigned int)3600)
#define CYCLES_OF_SLEEP_S ((unsigned int)60)

#endif

#define TEMP_SETPOINT_INC   5
#define TEMP_SETPOINT_MAX   220
#define TEMP_SETPOINT_MIN   130
#define TEMP_SETPOINT_OFF   0

#define STOP_STR ((const char*)"STOP")


typedef enum
{
    PB_IDLE = 0,
    PB_DEBOUCE,
    PB_SHORT_CLICK_CONFIRMED,
    PB_LONG_CLICK_CONFIRMED,
    PB_VERYLONG_CLICK_CONFIRMED
} t_push_button_state;

typedef void (*t_callback)(void);
typedef void (*t_click_callback)(t_push_button_state);

/* Function prototypes */
void rsi_red();
void go_to_sleep();

void thermo_logic_time_to_off();
void oled_update_time_to_off();
void click_time_to_off(t_push_button_state);

void thermo_logic_time_to_on();
void oled_update_time_to_on();
void click_time_to_on(t_push_button_state);

void thermo_logic_temp_setpoint();
void oled_update_temp_setpoint();
void click_temp_setpoint(t_push_button_state);

/***********************/

typedef enum
{
    MODE_TIME = 0,
    MODE_TEMP
} t_thermo_mode;

typedef enum
{
    HEATER_OFF = 0,
    HEATER_ON
} t_heater_status;

typedef enum
{
    POWER_ON = 0,
    POWER_SAVE
} t_thermo_power_mode;

typedef struct
{
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

typedef struct
{
    t_callback thermo_logic;
    t_callback oled_update;
    t_click_callback click_callback;
} t_state_functions;

static t_state_functions state_time_to_off{
    thermo_logic_time_to_off,
    oled_update_time_to_off,
    click_time_to_off};

static t_state_functions state_time_to_on{
    thermo_logic_time_to_on,
    oled_update_time_to_on,
    click_time_to_on};

static t_state_functions state_temp_setpoint{
    thermo_logic_temp_setpoint,
    oled_update_temp_setpoint,
    click_temp_setpoint};

static t_state_functions *state_current = &state_time_to_off;

typedef enum
{
    CYCLIC = 0, INT_EXT
} t_wake_up_cause;
volatile t_wake_up_cause wake_up_cause = CYCLIC;

volatile long timer_to_sleep = 0;
uint8_t remaining_sleep_cycles = 0;
// ========================== End of Header ====================================

void rsi_red()
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
    while (!Serial);

    init_io_pins();

    LED_ON;

    init_radio();
    init_flash();
    init_oled();

    LED_OFF;

    sample_data();
    
    td.setpoint = (int)(td.temperature/10)*10;
    td.remaining_time_s = TIMER_DISABLED;
    set_thermo_state(&state_time_to_off);
    state_current->oled_update();
    timer_to_sleep = millis() + TIMEOUT_TO_SLEEP_MS;
}

void loop()
{
    //    if (radio.receiveDone()) {
    //       CheckForWirelessHEX(radio, flash, false);
    //    }

    if (td.power_mode == POWER_SAVE) {
        during_power_save();
    }
    else {
        during_power_on();
    }
}

void click_time_to_off(t_push_button_state click_type)
{
    if (click_type == PB_SHORT_CLICK_CONFIRMED) {
        td.remaining_time_s += TIME_INCREMENT_S;
        if (td.remaining_time_s > TIME_TO_OFF_MAX_S) {
            td.remaining_time_s = TIMER_DISABLED;
        }
    }
    else if (click_type == PB_LONG_CLICK_CONFIRMED) {
        oled.clear();
        set_thermo_state(&state_time_to_on);
    }
    else if (click_type == PB_VERYLONG_CLICK_CONFIRMED) {
        /* TBD */
    }
    else {
        /* Nothing */
    }
}

void click_time_to_on(t_push_button_state click_type)
{
    if (click_type == PB_SHORT_CLICK_CONFIRMED) {
        td.remaining_time_s += TIME_INCREMENT_S;
        if (td.remaining_time_s > TIME_TO_ON_MAX_S) {
            td.remaining_time_s = TIMER_DISABLED;
        }
    }
    else if (click_type == PB_LONG_CLICK_CONFIRMED) {
        oled.clear();
        set_thermo_state(&state_temp_setpoint);
    }
    else if (click_type == PB_VERYLONG_CLICK_CONFIRMED) {
        /* TBD */
    }
    else {
        /* Nothing */
    }
}

void click_temp_setpoint(t_push_button_state click_type)
{
    if (click_type == PB_SHORT_CLICK_CONFIRMED) {
        if(td.setpoint == TEMP_SETPOINT_OFF) {
            td.setpoint = TEMP_SETPOINT_MIN;
        }
        else {
            td.setpoint += TEMP_SETPOINT_INC;

            if (td.setpoint == TEMP_SETPOINT_OFF) {
                td.setpoint = TEMP_SETPOINT_MIN;
            }
            else if (td.setpoint > TEMP_SETPOINT_MAX) {
                td.setpoint = TEMP_SETPOINT_OFF;
            }
            else {
                /* Nothing */
            }
        }
    }
    else if (click_type == PB_LONG_CLICK_CONFIRMED) {
        oled.clear();
        set_thermo_state(&state_time_to_off);
    }
    else if (click_type == PB_VERYLONG_CLICK_CONFIRMED) {
        /* TBD */
    }
    else {
        /* Nothing */
    }
}

void heater_on()
{
    if (td.heater_status == HEATER_ON)
        return;

    td.heater_status = HEATER_ON;

#ifdef USE_DEBUG
    LED_ON;
#else
    do_relay_pulse();
#endif
}

void heater_off()
{
    if (td.heater_status == HEATER_OFF)
        return;

    td.heater_status = HEATER_OFF;
    
#ifdef USE_DEBUG
    LED_OFF;
#else
    do_relay_pulse();
#endif
}

//-------------- Thermostat Logic Section --------------

void thermo_logic_time_to_off()
{
    if (td.remaining_time_s == TIMER_DISABLED || 
       (td.remaining_time_s == TIME_ZERO)) {
        heater_off();
        td.remaining_time_s = TIMER_DISABLED;
    }
    else {
        heater_on();
    }
}

void thermo_logic_time_to_on()
{
    if (td.remaining_time_s == TIME_ZERO) {
        heater_on();
        
        /* The following code is used to turn OFF the heater 
         * at some point. If not used, heater would be ON forever! */
        set_thermo_state(&state_time_to_off);
        td.remaining_time_s = TIME_TO_ON_MAX_TIME_AFTER_S;
    }
    else {
        heater_off();
    }
}

void thermo_logic_temp_setpoint()
{
    uint16_t real_setpoint = td.setpoint;

    if (td.heater_status == HEATER_ON) {
        real_setpoint += 1;
    }
    else {
        /* Nothing */
    }

    if (td.temperature >= real_setpoint) {
        heater_off();
    }
    else if (td.temperature < td.setpoint) {
        heater_on();
    }
    else {
        /* Nothing */
    }
}
//------------------------------------------------------

void during_power_save()
{
    if (remaining_sleep_cycles == 0) {
        remaining_sleep_cycles = CYCLES_OF_SLEEP_S;
        
        sample_data();
        state_current->thermo_logic();
        
        tx_to_base();
    }
    else {
        if ((td.remaining_time_s != TIMER_DISABLED) &&
            (td.remaining_time_s > TIME_ZERO)) {
            td.remaining_time_s--;
            
            if (td.remaining_time_s == TIME_ZERO) {
                remaining_sleep_cycles = 0;
            }
        }

        if (remaining_sleep_cycles != 0)
            remaining_sleep_cycles--;

        go_to_sleep();
    }
}

void during_power_on()
{
    static long long timer_1s = millis() + 1000;
    
    if (millis() > timer_1s) {
        timer_1s = millis() + 1000;

        if ((td.remaining_time_s != TIMER_DISABLED) &&
            (td.remaining_time_s > TIME_ZERO))
            td.remaining_time_s--;
    }

    read_and_debounce_pushbutton();

    if (millis() > timer_to_sleep) {
        /* encapsular a una funcio */
        state_current->thermo_logic();
        tx_to_base();
        remaining_sleep_cycles = CYCLES_OF_SLEEP_S;
        td.power_mode = POWER_SAVE;
    }
}

void set_thermo_state(t_state_functions *new_state)
{
    state_current = new_state;
    td.remaining_time_s = TIMER_DISABLED;
}

void get_time_formatted(char *in_buff)
{
    uint8_t hours = 0;
    uint8_t minutes = 0;
    
    if(td.remaining_time_s != TIMER_DISABLED) {
        hours = td.remaining_time_s / 3600;
        minutes = (td.remaining_time_s % 3600) / 60;
        sprintf(in_buff, "%d:%.2d", hours, minutes);
    }
    else {
        sprintf(in_buff, STOP_STR);
    }
}

void oled_update_time_to_off()
{
    char buff[17];
    
    oled.home();
    oled.set2X();

    oled.println("Time To OFF");
    
    oled.clearToEOL();
    get_time_formatted(buff);
    oled.println(buff);
}

void oled_update_time_to_on()
{
    char buff[17];
    
    oled.home();
    oled.set2X();

    oled.println("Time To ON");
    
    oled.clearToEOL();
    get_time_formatted(buff);
    oled.println(buff);
}

void oled_update_temp_setpoint()
{
    char buff[17];
    
    oled.home();
    oled.set2X();

    oled.println("Setpoint");
    sprintf(buff, "Real: %s", String((td.temperature/10.0),1).c_str());
    oled.println(buff);
    
    oled.clearToEOL();
    
    sprintf(buff, "Obj.: %s", (td.setpoint==0) ? STOP_STR : String((td.setpoint/10.0),1).c_str());
    oled.println(buff);
}

void sample_data()
{
    int safeguard_loop = 20;

    while ((DHT.read22(DHT_PIN) != DHTLIB_OK) && (safeguard_loop-- > 0))
        delay(50);

    td.temperature = DHT.temperature * 10;
    td.humidity = DHT.humidity * 10;
    td.vbat_mv = get_vbat_mv();
}

void tx_to_base()
{
    uint8_t tx_buff[TX_BUFF_LEN];
    
#ifdef USE_DEBUG
    char buff[16];    
    sprintf(buff, "%d:%d:%d:%d", td.heater_status, td.temperature, td.humidity, td.vbat_mv);
    DEBUGLN(buff);
#endif

    tx_buff[0] = td.temperature & 0x00FF;
    tx_buff[1] = (td.temperature >> 8);

    tx_buff[2] = td.humidity & 0x00FF;
    tx_buff[3] = (td.humidity >> 8);

    tx_buff[4] = td.setpoint & 0x00FF;
    tx_buff[5] = (td.setpoint >> 8);

    tx_buff[6] = td.vbat_mv & 0x00FF;
    tx_buff[7] = (td.vbat_mv >> 8);

    //    tx_buff[8] = last_sample.cycle_ms; // Ha de ser menor que 255, sinó overflow

    tx_buff[9] = (uint8_t)td.heater_status;

    // radio.sendWithRetry(GATEWAYID, tx_buff, TX_BUFF_LEN, 2, 40);
    radio.send(GATEWAYID, tx_buff, TX_BUFF_LEN);
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

    attachInterrupt(digitalPinToInterrupt(BUTTON_IN), rsi_red, LOW);
    // Enter power down state with ADC and BOD module disabled.
    // Wake up when wake up pin is low.
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
    //    delay(1000);
    /* ZZzzZZzzZZzz */

    // Disable external pin interrupt on wake up pin.
    detachInterrupt(digitalPinToInterrupt(BUTTON_IN));

    if (wake_up_cause == INT_EXT) {
        DEBUGLN("INT_EXT");

        // Ojo s'ha de testejar. Struct compare!!
        //        if(state_current == state_time_to_off) {
        //            heater_on();
        //        }

        timer_to_sleep = millis() + TIMEOUT_TO_SLEEP_MS;
        td.power_mode = POWER_ON;
        init_oled();
        state_current->oled_update();
    }
}

static uint16_t get_vbat_mv()
{
    analogReference(INTERNAL); // Referencia interna de 1.1V

    uint16_t adc_vbat = analogRead(A7);

    for (int i = 0; i < 10; i++) {
        adc_vbat = analogRead(A7);
        delay(1);
    }

    float vbat = map(adc_vbat, 0, 1023, 0, 1100); // Passem de la lectura 0-1023 de ADC a mV de 0-1100mV
    vbat *= 11; // 11 és el factor de divisió del divisor.
    vbat = vbat + MAGIC_VBAT_OFFSET_MV;

    return (uint16_t) vbat;
}

static void do_relay_pulse()
{
    pinMode(RELAY_TRIGGER, OUTPUT);
    delay(50);

    digitalWrite(RELAY_TRIGGER, LOW);
    delay(250);
    digitalWrite(RELAY_TRIGGER, HIGH);

    pinMode(RELAY_TRIGGER, INPUT_PULLUP);
}

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
        if (digitalRead(BUTTON_IN) == PB_PRESSED) {
            pb_state = PB_DEBOUCE;
            tick_time = millis();
        }
        else {
            /* Keep the current state */
        }
        break;

    case PB_DEBOUCE:
        if (digitalRead(BUTTON_IN) == PB_RELEASED) {
            if ((millis() - tick_time) > VERYLONG_CLICK_TIME_MS) {
                pb_state = PB_VERYLONG_CLICK_CONFIRMED;
            }
            else if ((millis() - tick_time) > LONG_CLICK_TIME_MS) {
                pb_state = PB_LONG_CLICK_CONFIRMED;
            }
            else if ((millis() - tick_time) > SHORT_CLICK_TIME_MS) {
                pb_state = PB_SHORT_CLICK_CONFIRMED;
            }
            else {
                pb_state = PB_IDLE;
            }
        }
        else {
            /* Keep the current state */
        }
        break;

    case PB_SHORT_CLICK_CONFIRMED:
    case PB_LONG_CLICK_CONFIRMED:
    case PB_VERYLONG_CLICK_CONFIRMED:
        /* Reset sleep timer because button is pressed */
        timer_to_sleep = millis() + TIMEOUT_TO_SLEEP_MS;

        if (digitalRead(BUTTON_IN) == PB_RELEASED) {
            state_current->click_callback(pb_state);
            state_current->oled_update();
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
    radio.initialize(FREQUENCY, NODEID, NETWORKID);
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
#endif
}
