// Arduino libraries
#include <SoftwareSerial.h>
#include <SPI.h>           //comes with Arduino IDE (www.arduino.cc)
#include <Wire.h>          //comes with Arduino IDE (www.arduino.cc)

// Custom Arduino libraries
#include "../lib/project_cfg.h"

#include "LowPower.h"           // https://github.com/rocketscream/Low-Power

/*-------------------------------- Defines -----------------------------------*/
#define APPNAME_STR "Base"
#define BUILD_STR "1.0"

#define WITH_RFM69
// #define WITH_SPIFLASH

#if defined(WITH_RFM69)
#include "RFM69.h"            //get it here: https://www.github.com/lowpowerlab/rfm69
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

#define NUM_KEYS_MAX 8
#define READ_LINE_TIMEOUT 1500
#define THERMO_QUEUE_CHANNEL "426969"

#define AP_SSID "MOVISTAR_B8E0"
#define AP_PWD "vEEVqnsVXAJmJPvvqxqq"

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

/*-------------------------- Routine Prototypes ------------------------------*/
AtResult Esp8266ConnectToAP(const char*  ssid, const char* pwd);
AtResult Esp8266Reset(void);

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
bool ReadLine(SoftwareSerial *s, String *line_str, uint16_t timeout_ms=READ_LINE_TIMEOUT);
AtResult ReadUntilOkOrError(SoftwareSerial *s);
int ReadLinesUntilToken(SoftwareSerial* s, const char* token_str, String* out_line, uint16_t timeout_ms=READ_LINE_TIMEOUT);

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
t_power_state power_state;
t_system_state system_state;
t_wake_up_cause wake_up_cause;

String host_str("Host: api.thingspeak.com\r\n");
String close_str("Connection: close\r\n\r\n\r\n");

struct {
    bool is_new_data;
    uint16_t mode;
    uint16_t parameter;
} thermo_feedback;

bool is_first_ack = true;

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
#if defined(USE_DEBUG)
    Serial.begin(SERIAL_BR);
    while (!Serial);

    DEBUGVAL("AppName=", APPNAME_STR);
    DEBUGVAL("AppVersion=", BUILD_STR);
#endif

    InitIOPins();

    esp8266.begin(ESP8266_BR);
    while (!esp8266);

    LED_ON;
    
//    delay(1000);
//    DEBUGLN(Esp8266Reset());
    
    delay(1000);
    DEBUGLN(Esp8266ConnectToAP(AP_SSID, AP_PWD));

    InitRadio();
    InitFlash();

    delay(1000);
    GetNewThermostatData();

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

AtResult Esp8266Reset()
{
    String at_cmd = "AT+RST\r\n";
    String dummy = "";
    
    esp8266.println(at_cmd);
    DEBUGLN(at_cmd.c_str());
    
    if(ReadLinesUntilToken(&esp8266, "ready", &dummy, 10000) >= 0)
        return AT_OK;
    else
        return AT_ERROR;
}

AtResult Esp8266ConnectToAP(const char*  ssid, const char* pwd)
{
    String at_cmd = "";
    String dummy = "";
    
    at_cmd.concat("AT+CWJAP=\"");
    at_cmd.concat(ssid);
    at_cmd.concat("\",\"");
    at_cmd.concat(pwd);
    at_cmd.concat("\"\r\n");
    
    esp8266.println(at_cmd);
    DEBUGLN(at_cmd.c_str());
    
    if(ReadLinesUntilToken(&esp8266, "WIFI GOT IP", &dummy, 10000) >= 0)
        return AT_OK;
    else
        return AT_ERROR;
}

void SendFeedbackToThermostat()
{
    char out_buff[32];
    int idx = 0;
    
    if(thermo_feedback.is_new_data == true) {
        /* This is a workaround to avoid sending the first data, usually not valid */
        if(is_first_ack) {
            is_first_ack = false;
            return;
        }
        
        out_buff[idx++] = '1';
        
        out_buff[idx++] = lowByte(thermo_feedback.mode -1);
        
        out_buff[idx++] = lowByte(thermo_feedback.parameter);
        out_buff[idx++] = highByte(thermo_feedback.parameter);
    }
    else {
        out_buff[idx++] = '0';
    }
    
    if(radio.ACKRequested() == true) {
        DEBUGVAL("Sending ACK. idx=", idx);
        DEBUGVAL("thermo_feedback.is_new_data=", thermo_feedback.is_new_data);
        DEBUGVAL("thermo_feedback.mode=", thermo_feedback.mode);
        DEBUGVAL("thermo_feedback.parameter=", thermo_feedback.parameter);
        radio.sendACK(out_buff, idx);
    }
    else {
        DEBUGLN("Ack not requested.");
    }
}

AtResult GetLastFieldFromChannel(String channel_n, uint16_t field_n, uint16_t* result)
{
    AtResult ret = AT_ERROR;
    
    String get_str = "";
    get_str.concat("GET /channels/");
    get_str.concat(channel_n);
    get_str.concat("/fields/");
    get_str.concat(field_n);
    get_str.concat("/last HTTP/1.1\r\n");
    
    DEBUGLN(get_str.c_str());

    /* Start the sequence */
    /* 1. Open a connection to the host, port 80*/
    esp8266.print("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n");

    if(ReadUntilOkOrError(&esp8266) == AT_OK) {
        /* 2. Send the Tx bytes */
        esp8266.print("AT+CIPSEND=");
        esp8266.print(get_str.length() + host_str.length() + close_str.length());
        esp8266.print("\r\n");

        if(ReadUntilOkOrError(&esp8266) == AT_OK) {
            /* 3. Send the GET string */
            esp8266.print(get_str.c_str());
//            DEBUGLN(get_str);
            delay(5);

            /* 4. Send the host string */
            esp8266.print(host_str.c_str());
//            DEBUGLN(host_str);
            delay(5);

            /* 5. Send the close string */
            esp8266.print(close_str.c_str());
//            DEBUGLN(close_str);

            String myline = "";
            int idx_start = ReadLinesUntilToken(&esp8266, "$$", &myline);
            int idx_finish = myline.indexOf("CLOSED");

            if((idx_start >= 0) && (idx_finish > 0)) {
                uint16_t res = (uint16_t)myline.substring(idx_start+2, idx_finish).toInt();
                *result = res;
                ret = AT_OK;
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
        DEBUGVAL("thermo_feedback.is_new_data=", thermo_feedback.is_new_data);
    }
    else {
        DEBUGLN("ERROR. thermo_feedback.is_new_data");
    }
    
    if((thermo_feedback.is_new_data == true) && (ret == AT_OK)) {
        ret = GetLastFieldFromChannel(THERMO_QUEUE_CHANNEL, 2, &thermo_feedback.mode);
        DEBUGVAL("thermo_feedback.mode=", thermo_feedback.mode);
    }
    else {
        DEBUGLN("ERROR. thermo_feedback.mode");
    }
    
    if((thermo_feedback.is_new_data == true) && (ret == AT_OK)) {
        ret = GetLastFieldFromChannel(THERMO_QUEUE_CHANNEL, 3, &thermo_feedback.parameter);
        DEBUGVAL("thermo_feedback.parameter=", thermo_feedback.parameter);
    }
    else {
        DEBUGLN("ERROR. thermo_feedback.parameter");
    }
        
    if((thermo_feedback.is_new_data) == true && (ret == AT_OK)) {
        old_timestamp = timestamp;
        DEBUGLN("New data. Update timestamp!");
    }
    else {
        DEBUGLN("OLD data.");
    }
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
    DEBUGVAL("ACKRequested=", radio.ACK_REQUESTED);
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

bool ReadLine(SoftwareSerial *s, String *line_str, uint16_t timeout_ms=READ_LINE_TIMEOUT)
{
    long long t0 = millis();
    bool is_found = false;
    String mydata = "";
    
    while(!is_found && (DELTA_TIME(t0) < timeout_ms)) {
        if(s->available() > 0) {
            char c = s->read();
            mydata.concat(c);

            if(c == '\n') {
//                DEBUGVAL("newline=", mydata.c_str());
                is_found = true;
            }
        }
        else {
            /* Nothing */
        }
    }
    
    *line_str = String(mydata);
    
    return is_found;
}

AtResult ReadUntilOkOrError(SoftwareSerial *s)
{
    bool is_new_line = false;
    String new_line;
    
    do {
        new_line = "";
        is_new_line = ReadLine(s, &new_line);
                
        if(new_line.indexOf("OK") == 0) {
            return AT_OK;
        }
        else if(new_line.indexOf("ERROR") == 0) {
            return AT_ERROR;
        }
        else {
            /* Keep reading */
        }
    } while(is_new_line);
    
    return AT_UNKNOWN;
}

int ReadLinesUntilToken(SoftwareSerial* s, const char* token_str, String* out_line, uint16_t timeout_ms=READ_LINE_TIMEOUT)
{
    bool is_new_line = false;
    int token_idx = -1;
    
    do {
        is_new_line = ReadLine(s, out_line, timeout_ms);
        token_idx = out_line->indexOf(token_str);
        
//        DEBUGLN(out_line->c_str());
        
        if(token_idx >= 0)
            return token_idx;
        
    } while(is_new_line);
    
    /* Not found */
    return -1;
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

        /* 0. Close current connection, if any */
        esp8266.print("AT+CIPCLOSE\r\n");
        (void)ReadUntilOkOrError(&esp8266);
        
        /* Start the sequence */
        /* 1. Open a connection to the host*/
        esp8266.print("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n");
        
        if(ReadUntilOkOrError(&esp8266) == AT_OK) 
        {
            /* 2. Send the Tx bytes */
            esp8266.print("AT+CIPSEND=");
            esp8266.print(get_str.length() + host_str.length() + close_str.length());
            esp8266.print("\r\n");
            
            if(ReadUntilOkOrError(&esp8266) == AT_OK) 
            {
                /* 3. Send the GET string */
                esp8266.print(get_str);
                DEBUGLN(get_str);
                delay(5);

                /* 4. Send the host string */
                esp8266.print(host_str.c_str());
                DEBUGLN(host_str.c_str());
                delay(5);

                /* 5. Send the close string */
                esp8266.print(close_str.c_str());
                DEBUGLN(close_str.c_str());
                
                String dummy;
                ReadLinesUntilToken(&esp8266, "+IPD", &dummy);
            }
        }
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
