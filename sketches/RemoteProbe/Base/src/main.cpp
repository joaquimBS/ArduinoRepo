// Arduino libraries
#include <SoftwareSerial.h>
#include <SPI.h>           //comes with Arduino IDE (www.arduino.cc)
#include <Wire.h>          //comes with Arduino IDE (www.arduino.cc)

// Custom Arduino libraries
#include "../lib/project_cfg.h"

#include "LowPower.h"           // https://github.com/rocketscream/Low-Power

/*-------------------------------- Defines -----------------------------------*/
#define WITH_RFM69
// #define WITH_SPIFLASH

#if defined(WITH_RFM69)
#include "RFM69.h"            //get it here: https://www.github.com/lowpowerlab/rfm69
RFM69 radio;
#define GATEWAYID 255
#define NETWORKID 100
#define NODEID   250
#define FREQUENCY RF69_868MHZ
#define ENCRYPTKEY "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#endif

#if defined(WITH_SPIFLASH)
#include <SPIFlash.h>         //get it here: https://www.github.com/lowpowerlab/spiflash
SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)
#endif

#define ESP8266_BR 115200
SoftwareSerial serial_esp8266(SS_RX, SS_TX); // RX, TX

#define NUM_KEYS_MAX 8

/*------------------------------ Data Types ---------------------------------*/
typedef enum
{
    PS_POWER_SAVE = 0,
    PS_ON
} t_power_state;

typedef enum
{
    SS_MAIN_IDLE = 0
} t_system_state;

typedef enum
{
    CYCLIC = 0,
    INT_EXT
} t_wake_up_cause;

typedef enum
{
    PB_IDLE = 0,
    PB_DEBOUCE,
    PB_PUSH_CONFIRMED
} t_push_button_state;

volatile t_power_state power_state;
volatile t_system_state system_state;
volatile t_wake_up_cause wake_up_cause;

/*-------------------------- Routine Prototypes ------------------------------*/
void periodic_task();
void InitIOPins();
void InitRadio();
void InitFlash();
void power_state_on_entry();
void power_state_power_save_entry();
void go_to_sleep();
void wake_up_from_sleep();
void ReadAndDebouncePushbutton();
void ParseDataFromRadio();
void UploadDataToThingspeak();

/* ---------------------------- Global Variables ---------------------------- */
struct {
    uint8_t ch_id;
    uint16_t f1;
    uint16_t f2;
    uint16_t f3;
    uint16_t f4;
    uint16_t f5;
    uint16_t f6;
    uint16_t f7;
    uint16_t f8;
} rx_data;
    
String api_keys[NUM_KEYS_MAX] = {"CBJ575ETWD8PGJSP", /* Thermostat */
                                 "VKRW0LMQR0NBEGRT", /* SensoNevera */
                                 "EWJ62LC3K3HA1T4C", /* Remote Sensor 1 */
                                 "Z3QYK9GG6QRP0811", /* Remote Sensor 2 */
                                 "",
                                 "",
                                 "",
                                 ""};

// ========================== End of Header ====================================

/* -------------------------------- Routines -------------------------------- */
/**
 * This function will be called when the hardware INT1 is triggered.
 */
void rsi_int_1()
{
    power_state = PS_ON;
    system_state = SS_MAIN_IDLE;
}

void InitIOPins()
{
    SET_DIGITAL_PINS_AS_INPUTS();

    // -- Custom IO setup --
    pinMode(INT_RED, INPUT_PULLUP);
    pinMode(INFO_LED, OUTPUT);
    pinMode(SS_TX, OUTPUT);
    pinMode(SS_RX, INPUT);
}

void setup()
{
    Serial.begin(SERIAL_BR);
    while (!Serial);

    InitIOPins();

    serial_esp8266.begin(ESP8266_BR);
    while (!serial_esp8266);

    LED_ON;

    InitRadio();
    InitFlash();

    power_state = PS_ON;
    system_state = SS_MAIN_IDLE;

    DEBUGVAL("Receiving frequency MHz=", FREQUENCY == RF69_433MHZ ? 433 : FREQUENCY == RF69_868MHZ ? 868 : 915);

    LED_OFF;
}

void loop()
{
    switch (power_state) {
    case PS_ON:
        power_state_on_entry();
        break;

    case PS_POWER_SAVE:
        power_state_power_save_entry();
        break;

    default:
        break;
    }
}

void power_state_on_entry()
{
    ReadAndDebouncePushbutton();

    if (radio.receiveDone()) {
        LED_ON;
        ParseDataFromRadio();
        UploadDataToThingspeak();
        LED_OFF;
    }
}

void ParseDataFromRadio()
{
    rx_data.ch_id = radio.DATA[0];
    
    if(radio.DATALEN >= 3)
        rx_data.f1 = radio.DATA[1] + (radio.DATA[2] << 8);
    
    if(radio.DATALEN >= 5)
        rx_data.f2 = radio.DATA[3] + (radio.DATA[4] << 8);
    
    if(radio.DATALEN >= 7)
        rx_data.f3 = radio.DATA[5] + (radio.DATA[6] << 8);
    
    if(radio.DATALEN >= 9)
        rx_data.f4 = radio.DATA[7] + (radio.DATA[8] << 8);
    
    if(radio.DATALEN >= 11)
        rx_data.f5 = radio.DATA[9] + (radio.DATA[10] << 8);
    
    if(radio.DATALEN >= 13)
        rx_data.f6 = radio.DATA[11] + (radio.DATA[12] << 8);
    
    if(radio.DATALEN >= 15)
        rx_data.f7 = radio.DATA[13] + (radio.DATA[14] << 8);
    
    if(radio.DATALEN >= 17)
        rx_data.f8 = radio.DATA[15] + (radio.DATA[16] << 8);
    
    DEBUGVAL("rssi=", radio.RSSI);
    DEBUGVAL("ch_id=", rx_data.ch_id);
    DEBUGVAL("f1=", rx_data.f1);
    DEBUGVAL("f2=", rx_data.f2);
    DEBUGVAL("f3=", rx_data.f3);
    DEBUGVAL("f4=", rx_data.f4);
    DEBUGVAL("f5=", rx_data.f5);
    DEBUGVAL("f6=", rx_data.f6);
    DEBUGVAL("f7=", rx_data.f7);
    DEBUGVAL("f8=", rx_data.f8);
    DEBUGLN("");
}

void UploadDataToThingspeak()
{
    if(rx_data.ch_id < NUM_KEYS_MAX) {
        String get_str("GET /update?api_key=");

        get_str.concat(api_keys[rx_data.ch_id]);

        get_str.concat("&field1=");
        get_str.concat(String((rx_data.f1 / 1000.0), 2));

        get_str.concat("&field2=");
        get_str.concat(String((rx_data.f2 / 10.0), 1));

        get_str.concat("&field3=");
        get_str.concat(String((rx_data.f3 / 10.0), 1));

        get_str.concat("&field4=");
        get_str.concat(rx_data.f4);

        get_str.concat("&field5=");
        get_str.concat(rx_data.f5);

        get_str.concat("&field6=");
        get_str.concat(rx_data.f6);

        get_str.concat("&field7=");
        get_str.concat(rx_data.f7);

        get_str.concat("&field8=");
        get_str.concat(rx_data.f8);

        get_str.concat(" HTTP/1.1\r\n");

        String host_str("Host: api.thingspeak.com\r\n");
        String close_str("Connection: close\r\n\r\n\r\n");

        /* Start the sequence */
        /* 1. Open a connection to the host*/
        serial_esp8266.print(F("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n"));
        delay(2000);

        /* 2. Send the Tx bytes */
        serial_esp8266.print(F("AT+CIPSEND="));
        serial_esp8266.print(get_str.length() + host_str.length() + close_str.length());
        serial_esp8266.print("\r\n");
        delay(500);

        /* 3. Send the GET string */
        serial_esp8266.print(get_str);
        DEBUGLN(get_str);
        delay(500);

        /* 4. Send the host string */
        serial_esp8266.print(host_str);
        DEBUGLN(host_str);
        delay(500);

        /* 5. Send the close string */
        serial_esp8266.print(close_str);
        DEBUGLN(close_str);
    }
    else {
        /* Wrong channel_id */
    }
}

void button_pressed_callback()
{
    DEBUGLN("button_pressed_callback");

    LED_ON;
    UploadDataToThingspeak();
    LED_OFF;
}

/*
 * This function needs to be called very often.
 * It implements a FSM IDLE -> DEBOUNCE -> CONFIRM to read a switch value
 */
void ReadAndDebouncePushbutton()
{
    static t_push_button_state pb_state = PB_IDLE;
    static unsigned long tick_time = 0;

    switch (pb_state) {
    case PB_IDLE:
        if (digitalRead(INT_RED) == PB_PRESSED) {
            pb_state = PB_DEBOUCE;
            tick_time = millis();
        }
        else {
            /* Keep the current state */
        }
        break;

    case PB_DEBOUCE:
        if ((millis() - tick_time) > DEBOUNCE_TIME_MS) {
            pb_state = PB_PUSH_CONFIRMED;
        }
        else if (digitalRead(INT_RED) == PB_RELEASED) {
            pb_state = PB_IDLE;
        }
        else {
            /* Keep the current state */
        }
        break;

    case PB_PUSH_CONFIRMED:
        if (digitalRead(INT_RED) == PB_RELEASED) {
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

void go_to_sleep()
{
    // flash.sleep();   /* Only if it was awake. */
    radio.sleep();

    attachInterrupt(digitalPinToInterrupt(INT_RED), rsi_int_1, LOW);
    /*********************************/
    while (power_state == PS_POWER_SAVE)
        LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
    // delay(1000);
    // ZZzzZZzzZZzz....
    /*********************************/
    detachInterrupt(digitalPinToInterrupt(INT_RED));

    // Here we are awake and the pushbutton is un-pressed
    wake_up_from_sleep();
}

/* Wake up routine */
void wake_up_from_sleep()
{
    pinMode(INFO_LED, OUTPUT);
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
        flash.sleep();
    }
#endif
}
