// Arduino libraries
// #include <SoftwareSerial.h>
#include <SPI.h> //comes with Arduino IDE (www.arduino.cc)
#include <Wire.h> //comes with Arduino IDE (www.arduino.cc)

// Custom Arduino libraries
#include "main.h"

// #include "MemoryFree.h"
#include "SparkFunHTU21D.h"
#include "LowPower.h"   // https://github.com/rocketscream/Low-Power/archive/V1.6.zip
#include "RTClib.h"     // https://github.com/adafruit/RTClib/archive/1.2.0.zip

/*-------------------------------- Defines -----------------------------------*/
#define APPNAME_STR "SensoNevera"
#define BUILD_STR "1.2"

#define WITH_RFM69
#define WITH_SPIFLASH
//#define WITH_OLED

#define TX_BUFF_LEN_MAX ((uint8_t) 64)
#define MAGIC_VBAT_OFFSET_MV ((int8_t) -40) // Empirically obtained

#ifdef WITH_OLED
#define OLED_I2C_ADDR 0x3C
#include "SSD1306Ascii.h"   // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiAvrI2c.h"
SSD1306AsciiAvrI2c oled;
#define ENABLE_OLED_VCC  {pinMode(OLED_VCC, OUTPUT); digitalWrite(OLED_VCC, HIGH);}
#define DISABLE_OLED_VCC {pinMode(OLED_VCC, INPUT);  digitalWrite(OLED_VCC, HIGH);}
#endif

#if defined(WITH_RFM69)
#include "RFM69.h"         // https://github.com/lowpowerlab/RFM69
//#include <RFM69_ATC.h>     // https://github.com/lowpowerlab/RFM69
#include "RFM69_OTA.h"     // https://github.com/lowpowerlab/RFM69
RFM69 radio;
#define GATEWAYID 200
#define NETWORKID 100
#define NODEID 1
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

#define TIME_ZERO ((uint16_t)0)
#define TIMER_DISABLED ((uint16_t)-1)

#define DEFAULT_CYCLES_OF_SLEEP_S ((unsigned int)60)
#define DEFAULT_TIMEOUT_TO_SLEEP_S ((unsigned int)5)
#define MIN_CYCLES_OF_SLEEP_S ((unsigned int)20)
#define MAX_CYCLES_OF_SLEEP_S ((unsigned int)5*60)

#define OLED_LINE_SIZE_MAX 16

/*------------------------------ Data Types ---------------------------------*/
typedef enum
{
    POWER_SAVE = 0,
    POWER_ON
} ThermostatPowerMode;

typedef enum
{
    SLEEP_TICK = 0,
    PERIODIC_TASK,
    PUSHBUTTON
} WakeUpCause;

typedef struct
{
    ThermostatPowerMode power_mode;
    uint16_t remaining_time_s;
    uint16_t humidity;
    uint16_t temperature;
    uint16_t vbat_mv;
} ThermostatData;

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
uint16_t ReadVbatMv();
void ReadTempData();
void TransmitToBase();
void SampleData();
void FlashWakeup();
void FlashSleep();
void ResetTimerToSleep();
void DecreaseRemainingTimeTask();

void DuringPowerON();
void DuringPowerSave();

/* ---------------------------- Global Variables ---------------------------- */
HTU21D myHumidity;
RTC_DS1307 rtc;

ThermostatData td = {POWER_SAVE, 0, 0, 0, 0};

/* IMPORTANT to keep wake_up_cause init value to INT_EXT */
volatile WakeUpCause wake_up_cause = PUSHBUTTON;

uint64_t timer_to_sleep = 0;
uint16_t remaining_sleep_cycles = 0;

uint64_t sleep_task_init_time = 0;
uint32_t sleep_task_time = 0;

uint8_t skip_samples = 0; // flag to skip N temp readings

uint16_t sleep_cycles_config = DEFAULT_CYCLES_OF_SLEEP_S;
uint16_t timeout_to_sleep_config = DEFAULT_TIMEOUT_TO_SLEEP_S;
// ========================== End of Header ================================= //

/* -------------------------------- Routines -------------------------------- */
void RsiButtonCtrl()
{
    wake_up_cause = PUSHBUTTON;
}

void InitIOPins()
{
    SET_DIGITAL_PINS_AS_INPUTS();

    // -- Custom IO setup --
    pinMode(OLED_VCC, INPUT); // Not an error. See ENABLE_OLED_VCC
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
#if defined(USE_DEBUG)
    Serial.begin(SERIAL_BR);
    while (!Serial);

    DEBUGVAL("AppName=", APPNAME_STR);
    DEBUGVAL("AppVersion=", BUILD_STR);
#endif

    /* IO Pins need to be initialized prior to other peripherals start */
    InitIOPins();

    LED_ON;

    /* Peripherals initialization block */
    Wire.begin(); // CAUTION. What could happen if other Wire.begin() is issued?

    InitRadio();
    InitFlash();
//    InitOled();
//    InitRTC();
    
    myHumidity.begin();
    myHumidity.setResolution(USER_REGISTER_RESOLUTION_RH11_TEMP11);

    LED_OFF;
    
    // Make an initial data sampling.
    SampleData();

    // Set initial values to some variables
    td.remaining_time_s = TIMER_DISABLED;
    
    ResetTimerToSleep();
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

//------------------------------------------------------
/* TODO Refactor the task_time data and routines */
void DuringPowerSave()
{
    sleep_task_init_time = micros();

    SampleData();
    TransmitToBase();
    
    GoToSleep();
}

void DuringPowerON()
{
    if (radio.receiveDone()) {
        CheckForWirelessHEX(radio, flash, false);
    }

    if (millis() > timer_to_sleep) {
        /* Switch the device to power saving mode */
        td.power_mode = POWER_SAVE;
        skip_samples = 2;
    }
}

void SampleData()
{
    ReadTempData();
//    td.vbat_mv = ReadVbatMv();
}

void TransmitToBase()
{
    uint8_t tx_buff[TX_BUFF_LEN_MAX];
    uint8_t idx = 0;
    uint64_t t0 = micros();

    tx_buff[idx++] = NODEID; // This works as channel_id
    
    /* Field1 in Thingspeak channel */
    tx_buff[idx++] = lowByte(td.vbat_mv);
    tx_buff[idx++] = highByte(td.vbat_mv);

    tx_buff[idx++] = lowByte(td.temperature);
    tx_buff[idx++] = highByte(td.temperature);

    tx_buff[idx++] = lowByte(td.humidity);
    tx_buff[idx++] = highByte(td.humidity);
    
    tx_buff[idx++] = (uint8_t) 0;
    tx_buff[idx++] = (uint8_t) 0;
    
#if 1
    if(true == radio.sendWithRetry(GATEWAYID, tx_buff, idx)) {
        /* Attention, the following routine can turn ON or OFF the heater */
        DEBUGVAL("radio.DATALEN=", radio.DATALEN);
    }
    else {
        DEBUGLN("No ACK");
    }
#else
    radio.send(GATEWAYID, tx_buff, idx);
#endif
    
    DEBUGVAL("idx=", idx);
    DEBUGVAL("tx_time_us=", uint32_t(micros()-t0));
    (void)t0; // To mute a warning
}

void GoToSleep()
{
    radio.sleep();
    FlashSleep();
    
    DISABLE_RTC_VCC;
    
    Wire.end();
    digitalWrite(RELAY_FEEDBACK, LOW);

    wake_up_cause = SLEEP_TICK;
    remaining_sleep_cycles = sleep_cycles_config;
    
    attachInterrupt(digitalPinToInterrupt(BUTTON_CTRL), RsiButtonCtrl, LOW);
    
    sleep_task_time = micros() - sleep_task_init_time;
    DEBUGVAL("sleep_task_time=", sleep_task_time);
    
#ifdef USE_DEBUG
    /* To allow DEBUG traces to finish Tx */
    delay(50);
#endif
    
    // Enter power down state with ADC and BOD module disabled.
    // Wake up when wake up pin is low.
    while(wake_up_cause == SLEEP_TICK) {
        DecreaseRemainingTimeTask();
        LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
        /* ZZzzZZzzZZzz */
    }
    DEBUGVAL("wake_up_cause=", wake_up_cause);
    
    if (wake_up_cause == PUSHBUTTON) {
        detachInterrupt(digitalPinToInterrupt(BUTTON_CTRL));

        ResetTimerToSleep();
        td.power_mode = POWER_ON;
    }
    else if(wake_up_cause == PERIODIC_TASK) {
        Wire.begin();
    }
}

void DecreaseRemainingTimeTask()
{
    if ((td.remaining_time_s != TIMER_DISABLED) && (td.remaining_time_s > TIME_ZERO)) {
        td.remaining_time_s--;

        if (td.remaining_time_s == TIME_ZERO) {
            /* This is a shortcut to react sooner. */
            remaining_sleep_cycles = 0;
        }
        else {
            /* Nothing */
        }
    }

    if (remaining_sleep_cycles > 0) {
        remaining_sleep_cycles--;
    }
    else {
        wake_up_cause = PERIODIC_TASK;
    }
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
    if(skip_samples == 0) {
        float temp = myHumidity.readTemperature();
        int retries = 50;

        while((retries > 0) && ((temp == ERROR_I2C_TIMEOUT) || (temp == ERROR_BAD_CRC))) {
            delay(1);
            DEBUG(".");
            temp = myHumidity.readTemperature();

            /* ATTENTION. Possible lock! */
            retries--;
        }

        td.temperature = temp * 10;
        td.humidity = myHumidity.readHumidity() * 10;

        DEBUGVAL("temp=", td.temperature);
        DEBUGVAL("hum=", td.humidity);
    }
    else {
        skip_samples--;
    }
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
