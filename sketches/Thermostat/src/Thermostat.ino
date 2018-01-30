// Arduino libraries
// #include <SoftwareSerial.h>
#include <SPI.h>           //comes with Arduino IDE (www.arduino.cc)
#include <Wire.h>          //comes with Arduino IDE (www.arduino.cc)

// Custom Arduino libraries
#include "Thermostat.h"

// #include "MemoryFree.h"
#include "dht.h"
#include "LowPower.h"      // https://github.com/rocketscream/Low-Power
#include "RTClib.h"        // https://github.com/adafruit/RTClib/tree/1.2.0

/*-------------------------------- Defines -----------------------------------*/
#define WITH_RFM69
#define WITH_SPIFLASH
#define WITH_OLED

#define TX_BUFF_LEN ((uint8_t) 12)
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
#include <RFM69.h>         // https://github.com/lowpowerlab/RFM69
//#include <RFM69_ATC.h>     // https://github.com/lowpowerlab/RFM69
#include <RFM69_OTA.h>     // https://github.com/lowpowerlab/RFM69
RFM69 radio;
#define GATEWAYID 1
#define NETWORKID 100
#define NODEID 11
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
SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for windbond 4mbit flash
boolean flash_is_awake = false;
#endif

#define ENABLE_RTC_VCC  digitalWrite(RTC_VCC, HIGH)
#define DISABLE_RTC_VCC digitalWrite(RTC_VCC, LOW)

dht DHT;
RTC_DS1307 rtc;

#define SHORT_CLICK_TIME_MS 80
#define LONG_CLICK_TIME_MS 500
#define VERYLONG_CLICK_TIME_MS 3000
#define PB_PRESSED LOW  // Pin must be INPUT_PULLUP and pushbutton to GND
#define PB_RELEASED HIGH

#define SERIAL_BR 115200

#define TIME_ZERO ((uint16_t)0)
#define TIMER_DISABLED ((uint16_t)-1)

#ifdef USE_DEBUG
#define DEFAULT_TIMEOUT_TO_SLEEP_MS ((unsigned int)3000)
#define TIME_INCREMENT1_S ((unsigned int)10)
#define TIME_INCREMENT2_S ((unsigned int)10)
#define TIME_TO_OFF_MAX_S ((unsigned int)90)
#define TIME_TO_ON_MAX_S ((unsigned int)90)
#define DEFAULT_ON_AFTER_TIME_TO_ON_S ((unsigned int)5)
#define DEFAULT_CYCLES_OF_SLEEP_S ((unsigned int)20)
#else
#define DEFAULT_TIMEOUT_TO_SLEEP_MS ((unsigned int)10000)
#define TIME_INCREMENT1_S ((unsigned int)1801)
#define TIME_INCREMENT2_S ((unsigned int)901)
#define TIME_TO_OFF_MAX_S ((unsigned int)4*3601)
#define TIME_TO_ON_MAX_S ((unsigned int)12*3601)
#define DEFAULT_ON_AFTER_TIME_TO_ON_S ((unsigned int)3600)
#define DEFAULT_CYCLES_OF_SLEEP_S ((unsigned int)60)
#endif

#define TEMP_SETPOINT_INC 5
#define TEMP_SETPOINT_MAX 220
#define TEMP_SETPOINT_MIN 130
#define TEMP_SETPOINT_OFF 0

#define STOP_STR ((const char*)"STOP")

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
    POWER_ON = 0,
    POWER_SAVE
} ThermostatPowerMode;

typedef enum
{
    CYCLIC = 0,
    INT_EXT
} WakeUpCause;

typedef void (*VoidCallback)(void);
typedef void (*ClickCallback)(PushButtonState);

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
} StateFunctions;

/*-------------------------- Routine Prototypes ------------------------------*/
void RsiButtonCtrl();
void InitIOPins();
void InitRadio();
void InitFlash();
void InitOled();
void InitRTC();

void GoToSleep();
uint16_t ReadVbatMv();
void ReadTempData();
void TransmitToBase();
void SampleData();
void FlashWakeup();
void FlashSleep();
void ReadAndDebouncePushbutton();
void ResetTimerToSleep();
void HeaterOFF();
void HeaterON();
void SetThermoState(StateFunctions *new_state);
void GetTimeFormatted(char *in_buff);

void DuringPowerON();
void DuringPowerSave();

void ThermoLogicTimeToOff();
void OledUpdateTimeToOff();
void ClickTimeToOff(PushButtonState);

void ThermoLogicTimeToOn();
void OledUpdateTimeToOn();
void ClickTimeToOn(PushButtonState);

void ThermoLogicTempSetpoint();
void OledUpdateTempSetpoint();
void ClickTempSetpoint(PushButtonState);

/* ---------------------------- Global Variables ---------------------------- */
ThermostatData td = {0};
volatile WakeUpCause wake_up_cause = CYCLIC;

StateFunctions state_time_to_off{
    ThermoLogicTimeToOff,
    OledUpdateTimeToOff,
    ClickTimeToOff};

StateFunctions state_time_to_on{
    ThermoLogicTimeToOn,
    OledUpdateTimeToOn,
    ClickTimeToOn};

StateFunctions state_temp_setpoint{
    ThermoLogicTempSetpoint,
    OledUpdateTempSetpoint,
    ClickTempSetpoint};

StateFunctions *state_current = &state_time_to_off;

long timer_to_sleep = 0;
uint8_t remaining_sleep_cycles = 0;

/* Thermostat configuration (maybe a struct) */
uint16_t sleep_cycles_config = DEFAULT_CYCLES_OF_SLEEP_S;
uint16_t timeout_to_sleep_config = DEFAULT_TIMEOUT_TO_SLEEP_MS;
uint16_t on_after_time_to_on_config = DEFAULT_ON_AFTER_TIME_TO_ON_S;
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
    pinMode(BUTTON_CTRL, INPUT_PULLUP);
    pinMode(INFO_LED, OUTPUT);
    pinMode(RELAY_PLUS, OUTPUT);
    pinMode(RELAY_MINUS, OUTPUT);
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
    InitRTC();
    
    LED_OFF;

    // Make an initial data sampling.
    SampleData();
    
    // Set initial values to some variables
    td.setpoint = (int)(td.temperature/10)*10;
    td.remaining_time_s = TIMER_DISABLED;
    SetThermoState(&state_time_to_off);
    state_current->oled_update();
    
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

void ClickTimeToOff(PushButtonState click_type)
{
    if (click_type == PB_SHORT_CLICK_CONFIRMED) {
        td.remaining_time_s += TIME_INCREMENT1_S;
        if (td.remaining_time_s > TIME_TO_OFF_MAX_S) {
            td.remaining_time_s = TIMER_DISABLED;
        }
    }
    else if (click_type == PB_LONG_CLICK_CONFIRMED) {
        oled.clear();
        SetThermoState(&state_time_to_on);
    }
    else if (click_type == PB_VERYLONG_CLICK_CONFIRMED) {
        /* TBD */
    }
    else {
        /* Nothing */
    }
}

void ClickTimeToOn(PushButtonState click_type)
{
    if (click_type == PB_SHORT_CLICK_CONFIRMED) {
        td.remaining_time_s += TIME_INCREMENT2_S;
        if (td.remaining_time_s > TIME_TO_ON_MAX_S) {
            td.remaining_time_s = TIMER_DISABLED;
        }
    }
    else if (click_type == PB_LONG_CLICK_CONFIRMED) {
        oled.clear();
        SetThermoState(&state_temp_setpoint);
    }
    else if (click_type == PB_VERYLONG_CLICK_CONFIRMED) {
        /* TBD */
    }
    else {
        /* Nothing */
    }
}

void ClickTempSetpoint(PushButtonState click_type)
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
        SetThermoState(&state_time_to_off);
    }
    else if (click_type == PB_VERYLONG_CLICK_CONFIRMED) {
        /* TBD */
    }
    else {
        /* Nothing */
    }
}

void HeaterON()
{
    td.heater_status = HEATER_ON;

#ifdef USE_DEBUG
//    LED_ON;
#endif
    digitalWrite(RELAY_MINUS, LOW);
    digitalWrite(RELAY_PLUS, HIGH);
    
    delay(250);
    
    digitalWrite(RELAY_MINUS, LOW);
    digitalWrite(RELAY_PLUS, LOW);
}

void HeaterOFF()
{
    td.heater_status = HEATER_OFF;
    
#ifdef USE_DEBUG
//    LED_OFF;
#endif
    digitalWrite(RELAY_MINUS, HIGH);
    digitalWrite(RELAY_PLUS, LOW);
    
    delay(250);
    
    digitalWrite(RELAY_MINUS, LOW);
    digitalWrite(RELAY_PLUS, LOW);
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
        SetThermoState(&state_time_to_off);
        td.remaining_time_s = on_after_time_to_on_config;
    }
    else {
        HeaterOFF();
    }
}

void ThermoLogicTempSetpoint()
{
    uint16_t real_setpoint = td.setpoint;

    if (td.heater_status == HEATER_ON) {
        real_setpoint += 1;
    }
    else {
        /* Nothing */
    }

    if (td.temperature >= real_setpoint) {
        HeaterOFF();
    }
    else if (td.temperature < td.setpoint) {
        HeaterON();
    }
    else {
        /* Nothing */
    }
}
//------------------------------------------------------

void DuringPowerSave()
{
    if (remaining_sleep_cycles == 0) {
        remaining_sleep_cycles = sleep_cycles_config;
        
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

        GoToSleep();
    }
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
        state_current->thermo_logic();
        TransmitToBase();
        remaining_sleep_cycles = sleep_cycles_config;
        td.power_mode = POWER_SAVE;
    }
}

void SetThermoState(StateFunctions *new_state)
{
    state_current = new_state;
    td.remaining_time_s = TIMER_DISABLED;
}

void GetTimeFormatted(char *in_buff)
{
    uint8_t hours = 0;
    uint8_t minutes = 0;
    
    if(td.remaining_time_s != TIMER_DISABLED) {
        hours = td.remaining_time_s / 3600;
        minutes = (td.remaining_time_s % 3600) / 60;
        sprintf(in_buff, "%d:%.2d h", hours, minutes);
    }
    else {
        sprintf(in_buff, "APAGAT", hours, minutes);
    }
}

void OledUpdateTimeToOff()
{
    char buff[17];
    
    oled.home();
    oled.set2X();

    oled.println("Encendre");
    oled.println("durant");

    GetTimeFormatted(buff);
    oled.clearToEOL();
    oled.println(buff);
    sprintf(buff,"%sC  %s%%", String((td.temperature/10.0),1).c_str(),
                                 String(td.humidity/10).c_str());
    oled.println(buff);
}

void OledUpdateTimeToOn()
{
    char buff[17];
    
    oled.home();
    oled.set2X();

    oled.println("Encendre");
    oled.println("en");

    GetTimeFormatted(buff);
    oled.clearToEOL();
    oled.println(buff);
    sprintf(buff,"%sC  %s%%", String((td.temperature/10.0),1).c_str(),
                                 String(td.humidity/10).c_str());
    oled.println(buff);
}

void OledUpdateTempSetpoint()
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

void SampleData()
{
    ReadTempData();
    td.vbat_mv = ReadVbatMv();
}

void TransmitToBase()
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

    tx_buff[8] = (uint8_t)td.mode;
    tx_buff[9] = (uint8_t)td.heater_status;
    
    tx_buff[10] = td.remaining_time_s & 0x00FF;
    tx_buff[11] = (td.remaining_time_s >> 8);

    // radio.sendWithRetry(GATEWAYID, tx_buff, TX_BUFF_LEN, 2, 40);
    radio.send(GATEWAYID, tx_buff, TX_BUFF_LEN);
}

void GoToSleep()
{
    FlashSleep();
    radio.sleep();
    
    DISABLE_OLED_VCC;
    DISABLE_RTC_VCC;
    pinMode(OLED_VCC, OUTPUT);
    pinMode(BUTTON_CTRL, INPUT_PULLUP);
    pinMode(INFO_LED, OUTPUT);

    wake_up_cause = CYCLIC;

    attachInterrupt(digitalPinToInterrupt(BUTTON_CTRL), RsiButtonCtrl, LOW);
    // Enter power down state with ADC and BOD module disabled.
    // Wake up when wake up pin is low.
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
    //    delay(1000);
    /* ZZzzZZzzZZzz */

    // Disable external pin interrupt on wake up pin.
    detachInterrupt(digitalPinToInterrupt(BUTTON_CTRL));

    if (wake_up_cause == INT_EXT) {
        DEBUGLN("INT_EXT");
        
        ResetTimerToSleep();
        td.power_mode = POWER_ON;
        
        InitOled();
        InitRTC();
        FlashWakeup();
        
        state_current->oled_update();
    }
}

uint16_t ReadVbatMv()
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

void ReadTempData()
{
    int safeguard_loop = 20;

    while ((DHT.read22(DHT_PIN) != DHTLIB_OK) && (safeguard_loop-- > 0))
        delay(50);

    /* To avoid use of floats, multiply values by 10 and use 'de */
    td.temperature = DHT.temperature * 10;
    td.humidity = DHT.humidity * 10;
}

/*
 * This function needs to be called often (once per tick)
 * It implements a FSM IDLE -> DEBOUNCE -> CONFIRM to read a button value
 */
void ReadAndDebouncePushbutton()
{
    static PushButtonState pb_state = PB_IDLE;
    static int pressed_button = 0;  /* 0 means NONE */
    static unsigned long tick_time = 0;

    switch (pb_state) {
    case PB_IDLE:
        if (digitalRead(BUTTON_CTRL) == PB_PRESSED) {
            pressed_button = BUTTON_CTRL;
        }
        else if (digitalRead(BUTTON_UP) == PB_PRESSED) {
            pressed_button = BUTTON_UP;
        }
        else if (digitalRead(BUTTON_DOWN) == PB_PRESSED) {
            pressed_button = BUTTON_DOWN;
        }
        else {
            pressed_button = 0;
        }
    
        if (pressed_button != 0) {
            pb_state = PB_DEBOUCE;
            tick_time = millis();
        }
        else {
            /* Keep the current state */
        }
        break;

    case PB_DEBOUCE:
        if (digitalRead(pressed_button) == PB_RELEASED) {
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
            /* Keep waiting button release */
        }
        break;

    case PB_SHORT_CLICK_CONFIRMED:
    case PB_LONG_CLICK_CONFIRMED:
    case PB_VERYLONG_CLICK_CONFIRMED:
        /* Reset sleep timer because button is pressed */
        ResetTimerToSleep();
        
        /* Maybe check what button was pressed?
         * There may be special buttons that are not tied to a state. */

        state_current->click_callback(pb_state);
        state_current->oled_update();
        pb_state = PB_IDLE;
        break;
    }
}

void ResetTimerToSleep()
{
    timer_to_sleep = millis() + timeout_to_sleep_config;
}

void InitRadio()
{
#ifdef WITH_RFM69
    radio.initialize(FREQUENCY, NODEID, NETWORKID);
    radio.setHighPower();
    radio.encrypt(ENCRYPTKEY);
    radio.sleep();
#endif
}

void InitFlash()
{
#ifdef WITH_SPIFLASH
    if (flash.initialize()) {
        flash_is_awake = true;
    }
#endif
}

void FlashSleep()
{
#ifdef WITH_SPIFLASH
    if(flash_is_awake == false)
        return;
    
    flash.sleep();
    flash_is_awake = false;
#endif
}

void FlashWakeup()
{
#ifdef WITH_SPIFLASH
    if(flash_is_awake == true)
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
    oled.setFont(Stang5x7);
    oled.clear();
#endif
}

void InitRTC()
{
    /* PRE: Wire.begin() need to be run somewhere before */
    ENABLE_RTC_VCC;
    delay(25);
    
    /* rtc.begin(); not necessary because it only runs Wire.begin() */
    
    if (false == rtc.initialized()) {
        DEBUGLN("RTC was NOT running. Setting current time.");
        // following line sets the RTC to the date & time this sketch was compiled
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    else {
        /* Nothing */
    }
    
    uint32_t unixtime = rtc.unixtime(); // should just work
    DEBUG("Unixtime: "); DEBUGLN(unixtime);
}
