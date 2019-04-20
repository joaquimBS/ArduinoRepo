// Arduino libraries
#include <SoftwareSerial.h>
#include <SPI.h>           //comes with Arduino IDE (www.arduino.cc)
#include <Wire.h>
#include <stdio.h>
#include <string.h>          //comes with Arduino IDE (www.arduino.cc)

// Custom Arduino libraries
#include "../lib/project_cfg.h"

#include "LowPower.h"           // https://github.com/rocketscream/Low-Power

/*-------------------------------- Defines -----------------------------------*/
#define APPNAME_STR "ThermoGateway"
#define BUILD_STR "1.1"

#define WITH_RFM69
// #define WITH_SPIFLASH

#if defined(WITH_RFM69)
#include "RFM69.h"            //get it here: https://www.github.com/lowpowerlab/rfm69
#include "RFM69registers.h"
RFM69 radio;
#define GATEWAYID 200
#define NETWORKID 100
#define NODEID   200
#define FREQUENCY RF69_868MHZ
#define ENCRYPTKEY "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#endif

#if defined(WITH_SPIFLASH)
#include <SPIFlash.h>         //get it here: https://www.github.com/lowpowerlab/spiflash
SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)
#endif

#define ESP8266_BR 9600 
SoftwareSerial esp8266(SS_RX, SS_TX); // RX, TX

#define MAX_READ_LINE 32
#define IO_BUFFER_LEN 140

#define NUM_KEYS_MAX 1
#define KEYS_LEN 16
#define READ_LINE_TIMEOUT 1500
#define THERMO_QUEUE_CHANNEL ((const char*)"426969")

#define HOST_STR F("Host: api.thingspeak.com\r\n")
#define HOST_STR_LEN 26

#define CLOSE_STR F("Connection: close\r\n\r\n\r\n")
#define CLOSE_STR_LEN 23

#define AP_SSID "CasaIoT"
#define AP_PWD "iot-de-casa"

#define HALT do {LED_OFF; delay(100); LED_ON; delay(100);} while(1)

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

typedef enum {
    AT_OK = 0,
    AT_ERROR,
    AT_UNKNOWN
} AtResult;

typedef struct {
    uint8_t ch_id;
    uint16_t f1;
    uint16_t f2;
    uint16_t f3;
    uint16_t f4;
    uint16_t f5;
    uint16_t f6;
    uint16_t f7;
    uint16_t f8;
} t_rx_data;

/*-------------------------- Routine Prototypes ------------------------------*/
AtResult Esp8266ConnectToAP(const char*  ssid, const char* pwd);
AtResult Esp8266Reset(void);

void SendFeedbackToThermostat(void);
void GetNewThermostatData(void);

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
t_rx_data rx_data {0, 0, 0, 0, 0, 0, 0, 0, 0};
    
const char* api_keys[NUM_KEYS_MAX] = {
    "CBJ575ETWD8PGJSP" /* Thermostat */
//    "VKRW0LMQR0NBEGRT", /* SensoNevera */
//    "EWJ62LC3K3HA1T4C", /* Remote Sensor 1 */
//    "Z3QYK9GG6QRP0811"  /* Remote Sensor 2 */
//    "XXXXXXXXXXXXXXXX",
//    "XXXXXXXXXXXXXXXX",
//    "XXXXXXXXXXXXXXXX",
//    "XXXXXXXXXXXXXXXX"
};

t_power_state power_state;
t_system_state system_state;
t_wake_up_cause wake_up_cause;

char io_buff[IO_BUFFER_LEN] = {0};

struct {
    bool is_new_data;
    uint16_t mode;
    uint16_t parameter;
} thermo_feedback;

bool is_first_ack = true;

// ========================== End of Header ====================================

/* ------------------------Static Routines ---------------------------------- */
static void ReadLine(SoftwareSerial *s, char new_line[MAX_READ_LINE], uint16_t timeout_ms=READ_LINE_TIMEOUT)
{
    long long deadline = millis() + timeout_ms;
    bool keep_reading = true;
    uint8_t idx = 0;
    
    memset(new_line, '\0', MAX_READ_LINE);
    
    while(keep_reading && (millis() < deadline)) {
        if(s->available() > 0) {
            char c = s->read();
            new_line[idx++] = c;
            
            if((MAX_READ_LINE == idx) || (c == '\n')) {
                new_line[idx-1] = '\0';
                keep_reading = false;
            }
        }
    }
}

static AtResult ReadUntilOkOrError(SoftwareSerial *s)
{
    AtResult retval = AT_UNKNOWN;
    char new_line[MAX_READ_LINE];
    
    do {
        ReadLine(s, new_line);
        
        if(NULL != strstr((const char*)new_line, (const char*)"OK")) {
            retval = AT_OK;
            DEBUGLN(F("AT_OK"));
        }
        else if(NULL != strstr((const char*)new_line, (const char*)"ERROR")) {
            retval = AT_ERROR;
            DEBUGLN(F("AT_ERROR"));
        }
        else {
            /* Keep reading */
        }
    } while(AT_UNKNOWN == retval);
    
    return retval;
}

static boolean ReadLinesUntilToken(SoftwareSerial* s, const char* token_str, uint16_t timeout_ms=READ_LINE_TIMEOUT)
{
    char *token_ptr = NULL;
    long long t0 = millis();
    
    /* In this implementation, the buffer may be local, 
     * since we are not interested in the result of strstr */
    char new_line[MAX_READ_LINE];
    
    DEBUGVAL(F("Reading until="), token_str);
    
    while((NULL == token_ptr) && (DELTA_TIME(t0) < timeout_ms)) {
        ReadLine(s, new_line, timeout_ms);
        token_ptr = strstr((const char*)new_line, token_str);
    }
    
    DEBUGVAL(F("token_found="), (NULL != token_ptr) ? "TRUE" : "FALSE");
    
    return (NULL != token_ptr);
}

static char* ReadLinesUntilToken(SoftwareSerial* s, char new_line[MAX_READ_LINE], const char* token_str, uint16_t timeout_ms=READ_LINE_TIMEOUT)
{
    char *token_ptr = NULL;
    long long t0 = millis();
    
    /* In this implementation, the buffer must be global, 
     * since we are interested in the result of strstr */
    
    DEBUGVAL(F("Reading until="), token_str);
    
    while((NULL == token_ptr) && (DELTA_TIME(t0) < timeout_ms)) {
        ReadLine(s, new_line, timeout_ms);
        token_ptr = strstr((const char*)new_line, token_str);
    }
    
    DEBUGVAL(F("token_found="), (NULL != token_ptr) ? "TRUE" : "FALSE");
    
    return token_ptr;
}

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
#if defined(USE_DEBUG)
    Serial.begin(SERIAL_BR);
    while (!Serial);

    DEBUGVAL(F("AppName="), APPNAME_STR);
    DEBUGVAL(F("AppVersion="), BUILD_STR);
#endif

    InitIOPins();
    
    esp8266.begin(ESP8266_BR);
    while (!esp8266);

    LED_ON;
    
    delay(2000);
    Esp8266Reset();
    
    Esp8266ConnectToAP(AP_SSID, AP_PWD); // join AP
    
    delay(5000);
    DEBUGLN(F("AT+GMR\r\n"));
    esp8266.println("AT+GMR\r\n");
    (void)ReadUntilOkOrError(&esp8266);
    
    delay(2000);
    DEBUGLN(F("AT+CIFSR\r\n"));
    esp8266.println("AT+CIFSR\r\n");
    (void)ReadUntilOkOrError(&esp8266);

    InitRadio();
    InitFlash();
    
    power_state = PS_ON;
    system_state = SS_MAIN_IDLE;

    DEBUGVAL(F("Receiving @ freqMHz="), FREQUENCY == RF69_433MHZ ? 433 : FREQUENCY == RF69_868MHZ ? 868 : 915);

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

AtResult Esp8266Reset()
{
    AtResult retval = AT_ERROR;
    
    while(AT_ERROR == retval) {
        DEBUGLN(F("AT+RST\r\n"));
        esp8266.println("AT+RST\r\n");

        if(true == ReadLinesUntilToken(&esp8266, "ready"))
            retval = AT_OK;
        else
            retval = AT_ERROR;
    }
    
    return retval;
}

AtResult Esp8266ConnectToAP(const char*  ssid, const char* pwd)
{
    snprintf(io_buff, IO_BUFFER_LEN, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pwd);
    DEBUGLN(io_buff);
    esp8266.println(io_buff);
    
    if(true == ReadLinesUntilToken(&esp8266, "WIFI GOT IP", 10000)) {
        return AT_OK;
    }
    else
        return AT_ERROR;
}

void power_state_on_entry()
{
    ReadAndDebouncePushbutton();

    if (radio.receiveDone()) {
        LED_ON;
    
        ParseDataFromRadio();
        SendFeedbackToThermostat();
        UploadDataToThingspeak();
        GetNewThermostatData();
        
        LED_OFF;
    }
}

void SendFeedbackToThermostat()
{
    char tx_buff[32];
    int idx = 0;
    
    if(thermo_feedback.is_new_data == true) {
        /* This is a workaround to avoid sending the first data, usually not valid */
        if(is_first_ack) {
            is_first_ack = false;
            return;
        }
        
        tx_buff[idx++] = '1';
        tx_buff[idx++] = lowByte(thermo_feedback.mode -1);
        tx_buff[idx++] = lowByte(thermo_feedback.parameter);
        tx_buff[idx++] = highByte(thermo_feedback.parameter);
    }
    else {
        DEBUGLN(F("Nothing new to send"));
        tx_buff[idx++] = '0';
    }
    
    if(radio.ACKRequested() == true) {
        DEBUGVAL(F("Sending ACK. idx="), idx);
        DEBUGVAL(F("thermo_feedback.is_new_data="), thermo_feedback.is_new_data);
        DEBUGVAL(F("thermo_feedback.mode="), thermo_feedback.mode);
        DEBUGVAL(F("thermo_feedback.parameter="), thermo_feedback.parameter);
        radio.sendACK(tx_buff, idx);
    }
    else {
        DEBUGLN("Ack not requested.");
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
    
//    if(radio.DATALEN >= 17)
//        rx_data.f8 = radio.DATA[15] + (radio.DATA[16] << 8);
    rx_data.f8 = radio.RSSI;
    
    DEBUGVAL(F("rssi="), radio.RSSI);
    DEBUGVAL(F("ACKRequested="), radio.ACK_REQUESTED);
    DEBUGVAL(F("ch_id="), rx_data.ch_id);
    DEBUGVAL(F("f1="), rx_data.f1);
    DEBUGVAL(F("f2="), rx_data.f2);
    DEBUGVAL(F("f3="), rx_data.f3);
    DEBUGVAL(F("f4="), rx_data.f4);
    DEBUGVAL(F("f5="), rx_data.f5);
    DEBUGVAL(F("f6="), rx_data.f6);
    DEBUGVAL(F("f7="), rx_data.f7);
    DEBUGVAL(F("f8="), rx_data.f8);
    DEBUGLN("");
}

void UploadDataToThingspeak()
{
    if(rx_data.ch_id >= NUM_KEYS_MAX) {
        HALT;
    }
    else {
        char f1_str[8] = {0};
        char f2_str[8] = {0};
        char f3_str[8] = {0};
        
        dtostrf((rx_data.f1 / 1000.0), 4, 2, f1_str);
        dtostrf((rx_data.f2 / 10.0),   4, 1, f2_str);
        dtostrf((rx_data.f3 / 10.0),   4, 1, f3_str);
        
        snprintf(
            io_buff,
            IO_BUFFER_LEN,
            "GET /update?api_key=%s&field1=%s&field2=%s&field3=%s&field4=%d&field5=%d&field6=%du&field7=%d&field8=%d HTTP/1.1\r\n",
            api_keys[rx_data.ch_id],
            f1_str,
            f2_str,
            f3_str,
            rx_data.f4,
            rx_data.f5,
            rx_data.f6,
            rx_data.f7,
            rx_data.f8
        );
        
        DEBUGVAL(F("Prepared to send="), io_buff);

        /* 0. Close current connection, if any */
        DEBUG(F("AT+CIPCLOSE\r\n"));
        esp8266.print("AT+CIPCLOSE\r\n");
        (void)ReadUntilOkOrError(&esp8266);
        
        /* Start the sequence */
        /* 1. Open a connection to the host*/
        DEBUG(F("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n"));
        esp8266.print("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n");
        
        if(ReadUntilOkOrError(&esp8266) == AT_OK) {
            /* 2. Send the Tx bytes */
            DEBUG(F("AT+CIPSEND="));
            esp8266.print("AT+CIPSEND=");

            DEBUG(strlen(io_buff) + HOST_STR_LEN + CLOSE_STR_LEN);
            esp8266.print(strlen(io_buff) + HOST_STR_LEN + CLOSE_STR_LEN);
            
            DEBUG(F("\r\n"));
            esp8266.print("\r\n");
            
            if(ReadUntilOkOrError(&esp8266) == AT_OK) {
                /* 3. Send the GET string */
                DEBUG(io_buff);
                esp8266.print(io_buff);
                delay(5);

                /* 4. Send the host string */
                DEBUG(HOST_STR);
                esp8266.print(HOST_STR);
                delay(5);

                /* 5. Send the close string */
                DEBUG(CLOSE_STR);
                esp8266.print(CLOSE_STR);
                
                esp8266.flush();
                
                (void)ReadLinesUntilToken(&esp8266, "CLOSED");
            }
        }
    }
}

AtResult GetLastFieldFromChannel(const char* channel_n_str, uint16_t field_n, uint16_t* result)
{
    AtResult ret = AT_ERROR;
    
    /* Start the sequence */
    /* 1. Open a connection to the host, port 80*/
    DEBUG(F("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n"));
    esp8266.print("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n");

    if(ReadUntilOkOrError(&esp8266) == AT_OK) {
        snprintf(io_buff, IO_BUFFER_LEN, "GET /channels/%s/fields/%d/last HTTP/1.1\r\n", channel_n_str, field_n);
        
        /* 2. Send the Tx bytes */
        DEBUG(F("AT+CIPSEND="));
        esp8266.print("AT+CIPSEND=");
        
        DEBUG(strlen(io_buff) + HOST_STR_LEN + CLOSE_STR_LEN);
        esp8266.print(strlen(io_buff) + HOST_STR_LEN + CLOSE_STR_LEN);
        
        DEBUG(F("\r\n"));
        esp8266.print("\r\n");

        if(ReadUntilOkOrError(&esp8266) == AT_OK) {
            /* 3. Send the GET string */
            DEBUG(io_buff);
            esp8266.print(io_buff);
            delay(5);

            /* 4. Send the host string */
            DEBUG(HOST_STR);
            esp8266.print(HOST_STR);
            delay(5);

            /* 5. Send the close string */
            DEBUG(CLOSE_STR);
            esp8266.print(CLOSE_STR);
            esp8266.flush();

            char myline[MAX_READ_LINE] = {0};
            char* token_start = ReadLinesUntilToken(&esp8266, myline, "$$", 5000);
            
            if(NULL != token_start) {
                DEBUGLN(F("token_start OK"));
                char* token_finish = strstr((const char*)myline, "CLOSED");
                
                if(NULL != token_finish) {
                    DEBUGLN(F("token_finish OK"));
                    *token_finish = '\0'; // finalize the number
                    *result = atoi(token_start+2); // +2 to skip $$
                    ret = AT_OK;
                }
                else {
                    DEBUGLN(F("token_finish ERROR"));
                }
            }
        }
    }

    return ret;
}

void GetNewThermostatData()
{
    static uint64_t old_timestamp = 0;
    uint16_t timestamp = 0;
    AtResult ret = AT_ERROR;
    
    thermo_feedback.is_new_data = false;
            
    ret = GetLastFieldFromChannel(THERMO_QUEUE_CHANNEL, 1, &timestamp);
    if(AT_OK == ret) {
        thermo_feedback.is_new_data = (timestamp != old_timestamp);
        DEBUGVAL(F("thermo_feedback.is_new_data="), thermo_feedback.is_new_data);
    }
    else {
        DEBUGLN(F("ERROR. thermo_feedback.is_new_data"));
    }
    
    if((thermo_feedback.is_new_data == true) && (ret == AT_OK)) {
        ret = GetLastFieldFromChannel(THERMO_QUEUE_CHANNEL, 2, &thermo_feedback.mode);
        DEBUGVAL(F("thermo_feedback.mode="), thermo_feedback.mode);
    }
    else {
        DEBUGLN(F("ERROR. thermo_feedback.mode"));
    }
    
    if((thermo_feedback.is_new_data == true) && (ret == AT_OK)) {
        ret = GetLastFieldFromChannel(THERMO_QUEUE_CHANNEL, 3, &thermo_feedback.parameter);
        DEBUGVAL(F("thermo_feedback.parameter="), thermo_feedback.parameter);
    }
    else {
        DEBUGLN(F("ERROR. thermo_feedback.parameter"));
    }
        
    if((thermo_feedback.is_new_data) == true && (ret == AT_OK)) {
        old_timestamp = timestamp;
        DEBUGLN(F("New data. Update timestamp!"));
    }
    else {
        DEBUGLN(F("OLD data."));
    }
}

void button_pressed_callback()
{
    DEBUGLN(F("button_pressed_callback"));

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
    
    /* The following instructins overwrite the default bitrate */
//    radio.writeReg(REG_BITRATEMSB, RF_BITRATEMSB_200KBPS);
//    radio.writeReg(REG_BITRATELSB, RF_BITRATELSB_200KBPS);
    
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
