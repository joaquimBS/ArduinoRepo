// Arduino libraries
// #include <SoftwareSerial.h>
#include <SPI.h>           //comes with Arduino IDE (www.arduino.cc)
#include <Wire.h>          //comes with Arduino IDE (www.arduino.cc)

// Custom Arduino libraries
#include "Thermostat.h"

// #include "MemoryFree.h"
#include "DHT.h"        // https://github.com/adafruit/DHT-sensor-library/archive/1.3.0.zip
#include "LowPower.h"   // https://github.com/rocketscream/Low-Power/archive/V1.6.zip
#include "RTClib.h"     // https://github.com/adafruit/RTClib/archive/1.2.0.zip

/*-------------------------------- Defines -----------------------------------*/
#define WITH_RFM69
#define WITH_SPIFLASH
#define WITH_OLED

#define TX_BUFF_LEN_MAX ((uint8_t) 64)
#define MAGIC_VBAT_OFFSET_MV ((int8_t) -40) // Empirically obtained

#ifdef WITH_OLED
#define OLED_I2C_ADDR 0x3C
#include "SSD1306Ascii.h"   // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiAvrI2c.h"
SSD1306AsciiAvrI2c oled;
#define ENABLE_OLED_VCC  digitalWrite(OLED_VCC, HIGH)
#define DISABLE_OLED_VCC digitalWrite(OLED_VCC, LOW)
#endif

#if defined(WITH_RFM69)
#include "RFM69.h"         // https://github.com/lowpowerlab/RFM69
//#include <RFM69_ATC.h>     // https://github.com/lowpowerlab/RFM69
#include "RFM69_OTA.h"     // https://github.com/lowpowerlab/RFM69
RFM69 radio;
#define GATEWAYID 255
#define NETWORKID 100
#define NODEID 0
#define FREQUENCY RF69_868MHZ
#define ENCRYPTKEY "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#endif

#if defined(WITH_SPIFLASH)
//******************************************************************************
// flash(SPI_CS, MANUFACTURER_ID)
// SPI_CS          - CS pin attached to SPI flash chip (8 in case of Moteino)
// MANUFACTURER_ID - OPTIONAL, 0x1F44 for adesto(ex atmel) 4mbit flash
//                             0xEF30 for windbond 4mbit flash
//                             0xEF40 for windbond 16/64mbit flash
//******************************************************************************
#include "SPIFlash.h"   // https://github.com/LowPowerLab/SPIFlash/archive/master.zip
SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for windbond 4mbit flash
boolean flash_is_awake = false;
#endif

#define ENABLE_RTC_VCC  digitalWrite(RTC_VCC, HIGH)
#define DISABLE_RTC_VCC digitalWrite(RTC_VCC, LOW)

DHT dht(DHT_PIN, DHT22);
RTC_DS1307 rtc;

#define SHORT_CLICK_TIME_MS 80
#define LONG_CLICK_TIME_MS 500
#define VERYLONG_CLICK_TIME_MS 3000
#define PB_PRESSED LOW  // Pin must be INPUT_PULLUP and pushbutton to GND
#define PB_RELEASED HIGH  // Pin must be INPUT_PULLUP and pushbutton to GND

#define TIME_ZERO ((uint16_t)0)
#define TIMER_DISABLED ((uint16_t)-1)

#ifdef USE_DEBUG
#define TIME_INCREMENT_1800_S ((unsigned int)5)
#define TIME_INCREMENT_900_S ((unsigned int)5)
#define TIME_INCREMENT_20_S ((unsigned int)5)
#define TIME_INCREMENT_5_S ((unsigned int)5)

#define MAX_TIME_TO_OFF_S ((unsigned int)180)
#define MAX_TIME_TO_ON_S ((unsigned int)180)

#define DEFAULT_ON_AFTER_TIME_TO_ON_S ((unsigned int)60)
#define MIN_ON_AFTER_TIME_TO_ON_S ((unsigned int)30)
#define MAX_ON_AFTER_TIME_TO_ON_S ((unsigned int)180)

#define DEFAULT_TIMEOUT_TO_SLEEP_S ((unsigned int)5)
#define MIN_TIMEOUT_TO_SLEEP_S ((unsigned int)5)
#define MAX_TIMEOUT_TO_SLEEP_S ((unsigned int)30)

#define DEFAULT_CYCLES_OF_SLEEP_S ((unsigned int)20)
#else
#define TIME_INCREMENT_1800_S ((unsigned int)1801)
#define TIME_INCREMENT_900_S ((unsigned int)901)
#define TIME_INCREMENT_20_S ((unsigned int)20)
#define TIME_INCREMENT_5_S ((unsigned int)5)

#define MAX_TIME_TO_OFF_S ((unsigned int)4*3601)
#define MAX_TIME_TO_ON_S ((unsigned int)12*3601)

#define DEFAULT_ON_AFTER_TIME_TO_ON_S ((unsigned int)3600)
#define MIN_ON_AFTER_TIME_TO_ON_S ((unsigned int)1800)
#define MAX_ON_AFTER_TIME_TO_ON_S ((unsigned int)3*3600)

#define DEFAULT_TIMEOUT_TO_SLEEP_S ((unsigned int)10)
#define MIN_TIMEOUT_TO_SLEEP_S ((unsigned int)5)
#define MAX_TIMEOUT_TO_SLEEP_S ((unsigned int)30)

#define DEFAULT_CYCLES_OF_SLEEP_S ((unsigned int)60)
#endif

#define MIN_CYCLES_OF_SLEEP_S ((unsigned int)20)
#define MAX_CYCLES_OF_SLEEP_S ((unsigned int)5*60)

#define TEMP_HYSTERESIS_RANGE 10   // remember 0.1C resolution
#define TEMP_SETPOINT_INC 5
#define TEMP_SETPOINT_MAX 220
#define TEMP_SETPOINT_MIN 150
#define TEMP_SETPOINT_OFF 0

#define STOP_STR ((const char*)"STOP")
#define OLED_LINE_SIZE_MAX 16

/*------------------------------ Data Types ---------------------------------*/
typedef enum
{
    PB_IDLE = 0,
    PB_DEBOUCE,
    PB_SHORT_CLICK_CONFIRMED,
    PB_LONG_CLICK_CONFIRMED,
    PB_VERYLONG_CLICK_CONFIRMED
} PushButtonState;

typedef enum
{
    MODE_TIME = 0,
    MODE_TEMP
} ThermostatMode;

typedef enum
{
    HEATER_OFF = 0,
    HEATER_ON
} HeaterStatus;

typedef enum
{
    POWER_SAVE = 0,
    POWER_ON
} ThermostatPowerMode;

typedef enum
{
    INT_EXT = 0,
    CYCLIC = 1
} WakeUpCause;

typedef void (*VoidCallback)(void);
typedef void (*ClickCallback)(uint8_t, PushButtonState);

typedef struct
{
    HeaterStatus heater_status;
    ThermostatMode mode;
    ThermostatPowerMode power_mode;
    uint16_t remaining_time_s;
    uint16_t humidity;
    uint16_t temperature;
    uint16_t setpoint;
    uint16_t vbat_mv;
} ThermostatData;

typedef struct
{
    VoidCallback thermo_logic;
    VoidCallback oled_update;
    ClickCallback click_callback;
} ThermoStateFunctions;

typedef struct
{
    uint8_t h;
    uint8_t m;
    uint8_t s;
} TimeHMS;

/*-------------------------- Routine Prototypes ------------------------------*/
void RsiButtonCtrl();
void InitIOPins();
void InitRadio();
void InitFlash();
void InitOled();
void InitRTC();

void GoToSleep();
HeaterStatus ReadHeaterStatus();
uint16_t ReadVbatMv();
void ReadTempData();
void TransmitToBase();
void SampleData();
void FlashWakeup();
void FlashSleep();
void ReadAndDebouncePushbutton();
void ResetTimerToSleep();
void HeaterON();
void HeaterOFF();
void SetThermoState(ThermoStateFunctions *new_state);
TimeHMS SecondsToHMS(uint16_t seconds);

void DuringPowerON();
void DuringPowerSave();

void OledEngineeringMode();

void ThermoLogicTimeToOff();
void OledStateTimeToOff();
void ClickTimeToOff(uint8_t, PushButtonState);

void ThermoLogicTimeToOn();
void OledStateTimeToOn();
void ClickTimeToOn(uint8_t, PushButtonState);

void ThermoLogicTempSetpoint();
void OledStateTempSetpoint();
void ClickTempSetpoint(uint8_t, PushButtonState);

void OledConfigTimeOnAfterTimeToOn();
void ClickTimeOnAfterTimeToOn(uint8_t, PushButtonState);

void OledConfigSleepTime();
void ClickSleepTime(uint8_t, PushButtonState);

void OledConfigTimeoutToSleep();
void ClickTimoutToSleep(uint8_t, PushButtonState);

/* ---------------------------- Global Variables ---------------------------- */
ThermostatData td = {HEATER_OFF, MODE_TIME, POWER_ON, 0, 0, 0, 0, 0};

/* IMPORTANT to keep wake_up_cause init value to INT_EXT */
volatile WakeUpCause wake_up_cause = INT_EXT;

ThermoStateFunctions thermo_state_time_to_off{
    ThermoLogicTimeToOff,
    OledStateTimeToOff,
    ClickTimeToOff};

ThermoStateFunctions thermo_state_time_to_on{
    ThermoLogicTimeToOn,
    OledStateTimeToOn,
    ClickTimeToOn};

ThermoStateFunctions thermo_state_temp_setpoint{
    ThermoLogicTempSetpoint,
    OledStateTempSetpoint,
    ClickTempSetpoint};

ThermoStateFunctions config_state_on_after_time_to_on{
    NULL_PTR,
    OledConfigTimeOnAfterTimeToOn,
    ClickTimeOnAfterTimeToOn};

ThermoStateFunctions config_state_sleep_time_s{
    NULL_PTR,
    OledConfigSleepTime,
    ClickSleepTime};

ThermoStateFunctions config_state_timeout_to_sleep{
    NULL_PTR,
    OledConfigTimeoutToSleep,
    ClickTimoutToSleep};

ThermoStateFunctions *state_current = &thermo_state_time_to_off;
ThermoStateFunctions *state_current_saved = (ThermoStateFunctions*) NULL_PTR;

long long timer_to_sleep = 0;
uint16_t remaining_sleep_cycles = 0;

/* Thermostat configuration (maybe a struct) */
uint16_t on_after_time_to_on_config = DEFAULT_ON_AFTER_TIME_TO_ON_S;
uint16_t sleep_cycles_config = DEFAULT_CYCLES_OF_SLEEP_S;
uint16_t timeout_to_sleep_config = DEFAULT_TIMEOUT_TO_SLEEP_S;
// ========================== End of Header ================================= //

/* -------------------------------- Routines -------------------------------- */
void RsiButtonCtrl()
{
    wake_up_cause = INT_EXT;
}

void InitIOPins()
{
    SET_DIGITAL_PINS_AS_INPUTS();

    // -- Custom IO setup --
    pinMode(OLED_VCC, OUTPUT);
    pinMode(RTC_VCC, OUTPUT);
    pinMode(BUTTON_CTRL, INPUT_PULLUP);
    pinMode(BUTTON_UP, INPUT_PULLUP);
    pinMode(BUTTON_DOWN, INPUT_PULLUP);
    pinMode(INFO_LED, OUTPUT);
    pinMode(RELAY_PLUS, OUTPUT);
    pinMode(RELAY_MINUS, OUTPUT);
    pinMode(RELAY_FEEDBACK, INPUT_PULLUP);
}

void setup()
{
    Serial.begin(SERIAL_BR);
    while (!Serial);

    /* IO Pins need to be initialized prior to other peripherals start */
    InitIOPins();

    LED_ON;

    /* Peripherals initialization block */
    Wire.begin(); // CAUTION. What could happen if other Wire.begin() is issued?

    InitRadio();
    InitFlash();
    InitOled();
//    InitRTC();
    
    dht.begin();

    LED_OFF;
    
    // Make an initial data sampling.
    SampleData();

    // Set initial values to some variables
    td.setpoint = (int) (td.temperature / 10)*10;
    td.remaining_time_s = TIMER_DISABLED;
    td.heater_status = ReadHeaterStatus();
    SetThermoState(&thermo_state_time_to_off);
    state_current->oled_update();
    
    ResetTimerToSleep();

    /* Initial safe condition of the heater */
    HeaterOFF();
}

void loop()
{
    if (td.power_mode == POWER_SAVE) {
        DuringPowerSave();
    }
    else {
        DuringPowerON();
    }
}

void ClickTimeToOff(uint8_t pb_id, PushButtonState click_type)
{
    if (click_type == PB_SHORT_CLICK_CONFIRMED) {
        if(pb_id == BUTTON_DOWN) {
            if (td.remaining_time_s > TIME_INCREMENT_1800_S) {
                td.remaining_time_s -= TIME_INCREMENT_1800_S;
            }
            else {
                td.remaining_time_s = TIME_ZERO;
            }
        }
        else if(pb_id == BUTTON_UP) {
            td.remaining_time_s += TIME_INCREMENT_1800_S;
            if (td.remaining_time_s > MAX_TIME_TO_OFF_S) {
                td.remaining_time_s = MAX_TIME_TO_OFF_S;
            }
        }
        else if(pb_id == BUTTON_CTRL) {
            oled.clear();
            SetThermoState(&thermo_state_time_to_on);
        }
        else {
            /* Nothing */
        }
    }
    else if (click_type == PB_LONG_CLICK_CONFIRMED) {
        /* TBD */
    }
    else if (click_type == PB_VERYLONG_CLICK_CONFIRMED) {
        /* TBD */
    }
    else {
        /* Nothing */
    }
}

void ClickTimeToOn(uint8_t pb_id, PushButtonState click_type)
{
    if (click_type == PB_SHORT_CLICK_CONFIRMED) {
        if(pb_id == BUTTON_DOWN) {
            if (td.remaining_time_s > TIME_INCREMENT_1800_S) {
                td.remaining_time_s -= TIME_INCREMENT_1800_S;
            }
            else {
                td.remaining_time_s = TIME_ZERO;
            }
        }
        else if(pb_id == BUTTON_UP) {
            td.remaining_time_s += TIME_INCREMENT_1800_S;
            if (td.remaining_time_s > MAX_TIME_TO_ON_S) {
                td.remaining_time_s = MAX_TIME_TO_ON_S;
            }
        }
        else if(pb_id == BUTTON_CTRL) {
            oled.clear();
            SetThermoState(&thermo_state_temp_setpoint);
        }
        else {
            /* Nothing */
        }
    }
    else if (click_type == PB_LONG_CLICK_CONFIRMED) {
        /* TBD */
    }
    else if (click_type == PB_VERYLONG_CLICK_CONFIRMED) {
        /* TBD */
    }
    else {
        /* Nothing */
    }
}

void ClickTempSetpoint(uint8_t pb_id, PushButtonState click_type)
{
    if (click_type == PB_SHORT_CLICK_CONFIRMED) {
        if(pb_id == BUTTON_DOWN) {
            if (td.setpoint > TEMP_SETPOINT_MIN) {
                td.setpoint -= TEMP_SETPOINT_INC;
            }
            else {
                td.setpoint = TEMP_SETPOINT_OFF;
            }
        }
        else if(pb_id == BUTTON_UP) {
            if (td.setpoint == TEMP_SETPOINT_OFF) {
                td.setpoint = TEMP_SETPOINT_MIN;
            }
            else if (td.setpoint > TEMP_SETPOINT_MAX) {
                td.setpoint = TEMP_SETPOINT_MAX;
            }
            else {
                td.setpoint += TEMP_SETPOINT_INC;
            }
        }
        else if(pb_id == BUTTON_CTRL) {
            oled.clear();
            SetThermoState(&thermo_state_time_to_off);
        }
        else {
            /* Nothing */
        }
    }
    else if (click_type == PB_LONG_CLICK_CONFIRMED) {
        /* TBD */
    }
    else if (click_type == PB_VERYLONG_CLICK_CONFIRMED) {
        /* TBD */
    }
    else {
        /* Nothing */
    }
}

void ClickTimeOnAfterTimeToOn(uint8_t pb_id, PushButtonState click_type)
{
    (void)pb_id;
    if (click_type == PB_SHORT_CLICK_CONFIRMED) {
        on_after_time_to_on_config += TIME_INCREMENT_900_S;
        if (on_after_time_to_on_config > MAX_ON_AFTER_TIME_TO_ON_S) {
            on_after_time_to_on_config = MIN_ON_AFTER_TIME_TO_ON_S;
        }
    }
    else if (click_type == PB_LONG_CLICK_CONFIRMED) {
        oled.clear();
        state_current = &config_state_sleep_time_s;
        /* Nothing */
    }
    else {
        /* Nothing */
    }
}

void ClickSleepTime(uint8_t pb_id, PushButtonState click_type)
{
    (void)pb_id;
    if (click_type == PB_SHORT_CLICK_CONFIRMED) {
        sleep_cycles_config += TIME_INCREMENT_20_S;
        if (sleep_cycles_config > MAX_CYCLES_OF_SLEEP_S) {
            sleep_cycles_config = MIN_CYCLES_OF_SLEEP_S;
        }
    }
    else if (click_type == PB_LONG_CLICK_CONFIRMED) {        
        oled.clear();
        state_current = &config_state_timeout_to_sleep;
        /* Nothing */
    }
    else {
        /* Nothing */
    }
}

void ClickTimoutToSleep(uint8_t pb_id, PushButtonState click_type)
{
    (void)pb_id;
    if (click_type == PB_SHORT_CLICK_CONFIRMED) {
        timeout_to_sleep_config += TIME_INCREMENT_5_S;
        if (timeout_to_sleep_config > MAX_TIMEOUT_TO_SLEEP_S) {
            timeout_to_sleep_config = MIN_TIMEOUT_TO_SLEEP_S;
        }
        ResetTimerToSleep();
    }
    else if (click_type == PB_LONG_CLICK_CONFIRMED) {
        oled.clear();
        state_current = &config_state_on_after_time_to_on;
        /* Nothing */
    }
    else {
        /* Nothing */
    }
}

void HeaterON()
{
    if(td.heater_status == HEATER_ON) {
        /* Keep the same heater status */
    }
    else {
#ifdef USE_DEBUG
        //    LED_ON;
#endif
        digitalWrite(RELAY_MINUS, LOW);
        digitalWrite(RELAY_PLUS, HIGH);

        delay(250);

        digitalWrite(RELAY_MINUS, LOW);
        digitalWrite(RELAY_PLUS, LOW);
    }
}

void HeaterOFF()
{
    if(td.heater_status == HEATER_OFF) {
        /* Keep the same heater status */
    }
    else {
#ifdef USE_DEBUG
    //    LED_OFF;
#endif
        digitalWrite(RELAY_MINUS, HIGH);
        digitalWrite(RELAY_PLUS, LOW);

        delay(250);

        digitalWrite(RELAY_MINUS, LOW);
        digitalWrite(RELAY_PLUS, LOW);
    }
}

//-------------- Thermostat Logic Section --------------

void ThermoLogicTimeToOff()
{
    if (td.remaining_time_s == TIMER_DISABLED ||
        (td.remaining_time_s == TIME_ZERO)) {
        HeaterOFF();
        td.remaining_time_s = TIMER_DISABLED;
    }
    else {
        HeaterON();
    }
}

void ThermoLogicTimeToOn()
{
    if (td.remaining_time_s == TIME_ZERO) {
        HeaterON();

        /* The following code is used to turn OFF the heater 
         * at some point. If not used, heater would be ON forever! */
        SetThermoState(&thermo_state_time_to_off);
        td.remaining_time_s = on_after_time_to_on_config;
    }
    else {
        HeaterOFF();
    }
}

void ThermoLogicTempSetpoint()
{
    uint16_t hysteresis_hi = td.setpoint;
    uint16_t hysteresis_lo = td.setpoint;

    /* This is to implement a TEMP_HYSTERESIS_RANGE hysteresis range. */
    hysteresis_hi += (TEMP_HYSTERESIS_RANGE / 2); // remember 0.1C resolution.
    hysteresis_lo -= (TEMP_HYSTERESIS_RANGE / 2); // remember 0.1C resolution.

    if (td.temperature >= hysteresis_hi) {
        HeaterOFF();
    }
    else if (td.temperature < hysteresis_lo) {
        HeaterON();
    }
    else {
        /* Nothing */
    }
}
//------------------------------------------------------
/* TODO Refactor the task_time data and routines */
static unsigned long init_time = 0;
static uint32_t task_time = 0;
static uint32_t periodic_sleep_time = 0;
static uint8_t periodic_sleep_samples = 0;
void DuringPowerSave()
{
    init_time = micros();
    if (remaining_sleep_cycles == 0) {
        DEBUGVAL("periodic_sleep_time_us=", periodic_sleep_time/periodic_sleep_samples);
        DEBUGVAL("task_time_us=", task_time);
        
        remaining_sleep_cycles = sleep_cycles_config;

        /* Once in a while, we keep track of the real heater status */
        td.heater_status = ReadHeaterStatus();
        
        SampleData();
        state_current->thermo_logic();
        TransmitToBase();
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
    }
    
    GoToSleep();
}

void DuringPowerON()
{
    static long long timer_1s = millis() + 1000;

    if (radio.receiveDone()) {
        CheckForWirelessHEX(radio, flash, false);
    }

    if (millis() > timer_1s) {
        timer_1s = millis() + 1000;

        if ((td.remaining_time_s != TIMER_DISABLED) &&
            (td.remaining_time_s > TIME_ZERO))
            td.remaining_time_s--;
    }

    ReadAndDebouncePushbutton();

    if (millis() > timer_to_sleep) {
        /* encapsular a una funcio */

        if (state_current_saved != NULL_PTR) {
            state_current = state_current_saved;
            state_current_saved = (ThermoStateFunctions*) NULL_PTR;
        }
        else {
            /* Nothing */
        }

        state_current->thermo_logic();
        TransmitToBase();
        remaining_sleep_cycles = sleep_cycles_config;
        td.power_mode = POWER_SAVE;
    }
}

void SetThermoState(ThermoStateFunctions *new_state)
{
    state_current = new_state;
    td.remaining_time_s = TIMER_DISABLED;
}

void OledEngineeringMode()
{
    char buff[OLED_LINE_SIZE_MAX];
    
    oled.clear();
    oled.home();
    
    snprintf(buff, OLED_LINE_SIZE_MAX, "%d mV", ReadVbatMv());
    oled.println(buff);
    snprintf(buff, OLED_LINE_SIZE_MAX, "%lu us", task_time);
    oled.println(buff);
}

void OledStateTimeToOff()
{
    char buff[OLED_LINE_SIZE_MAX];
    TimeHMS tdata = SecondsToHMS(td.remaining_time_s);

    oled.home();
    oled.set2X();

    oled.println("Encendre");
    oled.println("durant");

    oled.clearToEOL();
    snprintf(buff, OLED_LINE_SIZE_MAX, "%d:%02d h", tdata.h, tdata.m);    
    oled.println(td.remaining_time_s == TIMER_DISABLED ? STOP_STR : buff);
    
    snprintf(buff, OLED_LINE_SIZE_MAX, "%sC  %s%%", String((td.temperature / 10.0), 1).c_str(),
            String(td.humidity / 10).c_str());
    oled.println(buff);
}

void OledStateTimeToOn()
{
    char buff[OLED_LINE_SIZE_MAX];
    TimeHMS tdata = SecondsToHMS(td.remaining_time_s);

    oled.home();
    oled.set2X();

    oled.println("Encendre");
    oled.println("en");

    oled.clearToEOL();
    snprintf(buff, OLED_LINE_SIZE_MAX, "%d:%02d h", tdata.h, tdata.m);    
    oled.println(td.remaining_time_s == TIMER_DISABLED ? STOP_STR : buff);

    snprintf(buff, OLED_LINE_SIZE_MAX, "%sC  %s%%", String((td.temperature / 10.0), 1).c_str(),
            String(td.humidity / 10).c_str());
    oled.println(buff);
}

void OledStateTempSetpoint()
{
    char buff[OLED_LINE_SIZE_MAX];

    oled.home();
    oled.set2X();

    oled.println("Setpoint");
    snprintf(buff, OLED_LINE_SIZE_MAX, "Real: %s", String((td.temperature / 10.0), 1).c_str());
    oled.println(buff);

    oled.clearToEOL();
    snprintf(buff, OLED_LINE_SIZE_MAX, "Obj.: %s", (td.setpoint == 0) ? STOP_STR : String((td.setpoint / 10.0), 1).c_str());
    oled.println(buff);
}

void OledConfigTimeOnAfterTimeToOn()
{
    char buff[OLED_LINE_SIZE_MAX];
    TimeHMS tdata = SecondsToHMS(on_after_time_to_on_config);

    oled.home();
    oled.set2X();

    oled.println("ON after");
    oled.println("TimeToOn:");

    oled.clearToEOL();
    snprintf(buff, OLED_LINE_SIZE_MAX, "%d:%02d h", tdata.h, tdata.m);    
    oled.println(buff);
}

void OledConfigSleepTime()
{
    char buff[OLED_LINE_SIZE_MAX];
    TimeHMS tdata = SecondsToHMS(sleep_cycles_config);

    oled.home();
    oled.set2X();

    oled.println("Tx");
    oled.println("interval:");

    oled.clearToEOL();
    snprintf(buff, OLED_LINE_SIZE_MAX, "%dm%02ds", tdata.s);    
    oled.println(buff);
}

void OledConfigTimeoutToSleep()
{
    char buff[17];
    TimeHMS tdata = SecondsToHMS(timeout_to_sleep_config);

    oled.home();
    oled.set2X();

    oled.println("Timeout to");
    oled.println("sleep:");

    oled.clearToEOL();
    snprintf(buff, OLED_LINE_SIZE_MAX, "%d sec", tdata.s);    
    oled.println(buff);
}

void SampleData()
{
    ReadTempData();
    td.vbat_mv = ReadVbatMv();
}

void TransmitToBase()
{
    uint8_t tx_buff[TX_BUFF_LEN_MAX];
    uint8_t idx = 0;
    
    tx_buff[idx++] = NODEID; // This works as channel_id
    
    /* Field1 in Thingspeak channel */
    tx_buff[idx++] = lowByte(td.vbat_mv);
    tx_buff[idx++] = highByte(td.vbat_mv);

    tx_buff[idx++] = lowByte(td.temperature);
    tx_buff[idx++] = highByte(td.temperature);

    tx_buff[idx++] = lowByte(td.humidity);
    tx_buff[idx++] = highByte(td.humidity);
    
    tx_buff[idx++] = (uint8_t) td.heater_status;
    tx_buff[idx++] = (uint8_t) 0;

    tx_buff[idx++] = lowByte(td.setpoint);
    tx_buff[idx++] = highByte(td.setpoint);

    tx_buff[idx++] = lowByte(td.remaining_time_s);
    tx_buff[idx++] = highByte(td.remaining_time_s);
      
    tx_buff[idx++] = (uint8_t) td.mode;
    tx_buff[idx++] = (uint8_t) 0;
    
    tx_buff[idx++] = lowByte(task_time);
    tx_buff[idx++] = highByte(task_time);
    
    unsigned long t0 = micros();
    
#if 0
    radio.sendWithRetry(GATEWAYID, tx_buff, idx);
#else
    radio.send(GATEWAYID, tx_buff, idx);
#endif
    
    DEBUGVAL("idx=", idx);
    DEBUGVAL("tx_time_us=", micros()-t0);
}

void GoToSleep()
{
    if (wake_up_cause == INT_EXT) {
        FlashSleep();
        radio.sleep();

        DISABLE_OLED_VCC;
        DISABLE_RTC_VCC;

        digitalWrite(SDA, LOW);  // To minimize I2C current consumption during sleep.
        digitalWrite(SCL, LOW);
        pinMode(DHT_PIN, INPUT_PULLUP);

        wake_up_cause = CYCLIC;
    }
    
    attachInterrupt(digitalPinToInterrupt(BUTTON_CTRL), RsiButtonCtrl, LOW);
    
    /* TODO Refactor the task_time data and routines */
    /* Compute task times */
    if(remaining_sleep_cycles == sleep_cycles_config) {
        task_time = micros() - init_time;
        periodic_sleep_samples = 0;
        periodic_sleep_time = 0;
    }
    else {
        periodic_sleep_samples++;
        periodic_sleep_time += micros() - init_time;
    }
    
#ifdef USE_DEBUG
    /* To allow DEBUG msgs to finish Tx */
    delay(50);
#endif
    
    // Enter power down state with ADC and BOD module disabled.
    // Wake up when wake up pin is low.
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
//        delay(1000);
    /* ZZzzZZzzZZzz */

    // Disable external pin interrupt on wake up pin.
    detachInterrupt(digitalPinToInterrupt(BUTTON_CTRL));

    if (wake_up_cause == INT_EXT) {
        DEBUGLN("INT_EXT");

        ResetTimerToSleep();
        td.power_mode = POWER_ON;

        Wire.begin();

        InitOled();
//        InitRTC();
//        FlashWakeup();

        state_current->oled_update();
    }
}

HeaterStatus ReadHeaterStatus()
{
    /* Note the !. It is a negated signal */
    return (HeaterStatus)!digitalRead(RELAY_FEEDBACK);
}

uint16_t ReadVbatMv()
{
    analogReference(INTERNAL); // Referencia interna de 1.1V

    uint16_t adc_vbat = analogRead(VBAT_IN);

    for (int i = 0; i < 10; i++) {
        adc_vbat = analogRead(VBAT_IN);
        delay(1);
    }

    float vbat = map(adc_vbat, 0, 1023, 0, 1100); // Passem de la lectura 0-1023 de ADC a mV de 0-1100mV
    vbat *= 11; // 11 is the division value of the divisor
    vbat = vbat + MAGIC_VBAT_OFFSET_MV;

    return (uint16_t) vbat;
}

void ReadTempData()
{
    /* To avoid use of floats, multiply values by 10 and use 'de */
    td.temperature = dht.readTemperature(false, true) * 10;
    td.humidity = dht.readHumidity() * 10;
    
    DEBUG("temp:"); DEBUGLN(td.temperature);
    DEBUG("hum:"); DEBUGLN(td.humidity);
}

/*
 * This function needs to be called often (once per tick)
 * It implements a FSM IDLE -> DEBOUNCE -> CONFIRM to read a button value
 */
void ReadAndDebouncePushbutton()
{
    static PushButtonState pb_state = PB_IDLE;
    static int pb_id = 0; /* 0 means NONE */
    static long long tick_time = 0;

    switch (pb_state) {
    case PB_IDLE:
        if (digitalRead(BUTTON_CTRL) == PB_PRESSED) {
            pb_id = BUTTON_CTRL;
        }
        else if (digitalRead(BUTTON_UP) == PB_PRESSED) {
            pb_id = BUTTON_UP;
        }
        else if (digitalRead(BUTTON_DOWN) == PB_PRESSED) {
            pb_id = BUTTON_DOWN;
        }
        else {
            pb_id = 0;
        }

        if (pb_id != 0) {
            pb_state = PB_DEBOUCE;
            tick_time = millis();
        }
        else {
            /* Keep the current state */
        }
        break;

    case PB_DEBOUCE:
        if (digitalRead(pb_id) == PB_RELEASED) {
            /* Read the delta since PB was pressed. From higher to lower value */
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
                /* Return to Idle if pulse was too short */
                pb_state = PB_IDLE;
            }
        }
        else {
            /* Keep waiting button release, resetting timer to sleep */
            ResetTimerToSleep();
        }
        break;

    case PB_SHORT_CLICK_CONFIRMED:
    case PB_LONG_CLICK_CONFIRMED:
    case PB_VERYLONG_CLICK_CONFIRMED:
        /* Reset sleep timer because button is pressed */
        ResetTimerToSleep();
        
        /* Switch FSM: Thermo -> Config */
        if (pb_state == PB_VERYLONG_CLICK_CONFIRMED) {
            oled.clear();
            state_current_saved = state_current;
            state_current = &config_state_on_after_time_to_on;
            
            OledEngineeringMode();
            delay(1000);
        }
        else {
            /* If its click or long click, pass it to the state */
            state_current->click_callback(pb_id, pb_state);
        }
        
        state_current->oled_update();
        
        pb_state = PB_IDLE;
        break;
    }
}

TimeHMS SecondsToHMS(uint16_t seconds)
{
    TimeHMS time_data = {0,0,0};
    
    time_data.h = seconds / 3600;
    time_data.m = (seconds % 3600) / 60;
    time_data.s = (seconds % 3600) % 60;
    
    return time_data;
}

void ResetTimerToSleep()
{
    timer_to_sleep = millis() + (timeout_to_sleep_config * 1000);
}

void InitRadio()
{
#ifdef WITH_RFM69
    radio.initialize(FREQUENCY, NODEID, NETWORKID);
    radio.setHighPower();
    radio.encrypt(ENCRYPTKEY);
    radio.sleep();

    DEBUGLN("OK");
#endif
}

void InitFlash()
{
#ifdef WITH_SPIFLASH
    if (flash.initialize()) {
        flash_is_awake = true;
        DEBUGLN("OK");
    }
    else {
        DEBUGLN("ERROR");
    }
    FlashSleep();
#endif
}

void FlashSleep()
{
#ifdef WITH_SPIFLASH
    if (flash_is_awake == false)
        return;

    flash.sleep();
    flash_is_awake = false;
#endif
}

void FlashWakeup()
{
#ifdef WITH_SPIFLASH
    if (flash_is_awake == true)
        return;

    flash.wakeup();
    flash_is_awake = true;
#endif
}

void InitOled()
{
#ifdef WITH_OLED
    ENABLE_OLED_VCC;
    delay(500);

    oled.begin(&Adafruit128x64, OLED_I2C_ADDR);
    oled.setFont(System5x7);
    oled.clear();

    DEBUGLN("OK");
#endif
}

void InitRTC()
{
    /* PRE: Wire.begin() need to be run somewhere before */
    ENABLE_RTC_VCC;
    delay(25);

    /* rtc.begin(); // not necessary because it only runs Wire.begin() */

    if (false == rtc.isrunning()) {
        DEBUGLN("RTC was NOT running. Setting current time.");
        // following line sets the RTC to the date & time this sketch was compiled
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    else {
        /* Nothing */
    }

    DEBUGVAL("InitRTC OK. Unixtime: ", rtc.now().unixtime());
}
