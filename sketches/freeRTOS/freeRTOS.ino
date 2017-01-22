// #include "freeRTOS.h"

// Custom Arduino libaries
#include <dht.h>
#include "RTClib.h"			// https://github.com/adafruit/RTClib
// #include "IRremote.h"		// https://github.com/z3t0/Arduino-IRremote
#include <Arduino_FreeRTOS.h>
#include "semphr.h"  // add the FreeRTOS functions for Semaphores (or Flags).
//#include <MemoryFree.h>

#include <avr/power.h>
#include <avr/sleep.h>

//#include <SoftwareSerial.h>


enum {P0, P1, PIN_BTN_YLW, PIN_BTN_RED, DHTPIN, OLED_VCC, P6, P7, P8, P9, P10, P11, PIN_PWM, INFO_LED, PIN_COUNT};



#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

#define SERIAL_BPS 	115200

void InitRtc(int timeout_ms=500);

// OLED region
#define I2C_ADDRESS 0x3C
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"

// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only only one Task is accessing this resource at any time.
SemaphoreHandle_t xMySemaphore;
// SemaphoreHandle_t xWireSemaphore;

unsigned long tick = 0;
unsigned long dReadTime = 0;

// define two Tasks for DigitalRead & AnalogRead
void TaskFastPeriod( void *pvParameters );
void TaskTemperatureRead( void *pvParameters );
// void TaskTimeRead( void *pvParameters );

TaskHandle_t xHandle1 = NULL;
TaskHandle_t xHandle2 = NULL;
TaskHandle_t xHandle3 = NULL;

#define FREQ_1000_MS  1000
#define FREQ_500_MS   500
#define FREQ_100_MS   100

//SoftwareSerial mySerial(6, 5); // RX, TX

dht DHT;
SSD1306AsciiAvrI2c oled;
RTC_DS1307 rtc;
DateTime now;

void InitRtc(int timeout_ms)
{
  bool isRTCInit = false;
  char buff[20];

  // rtc.begin();   // Needed ONLY IF not done before.

  unsigned long t0 = millis();

  Serial.println("InitRtc...");
  
  while(!isRTCInit && ((millis() - t0) < timeout_ms) ) {
    if(rtc.isrunning())
      isRTCInit = true;
    else
      delay(25);
  }

  if(isRTCInit) {
    now = rtc.now();

    sprintf(buff, "RTC OK in %d ms.", millis() - t0);
    Serial.println(buff);

    sprintf(buff, "%04d/%02d/%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    Serial.println(buff);
  }
  else {
    Serial.println("RTC\tFail!");
  }
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

void InitOled()
{
  pinMode(OLED_VCC, OUTPUT);
  digitalWrite(OLED_VCC, LOW);

  delay(100);

  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(Stang5x7);
  oled.clear();
}

// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(SERIAL_BPS);
  while(!Serial);

  // InitOled();   // Initializes Wire interface, needed in InitRtc();
  // InitRtc();

  // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
  // because it is sharing a resource, such as the Serial port.
  // Semaphores should only be used whilst the scheduler is running, but we can set it up here.
  // if ( xMySemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  // {
  //   xMySemaphore = xSemaphoreCreateBinary();  // Create a mutex semaphore we will use to manage the Serial Port
  //   if ( ( xMySemaphore ) != NULL )
  //     xSemaphoreTake( xMySemaphore, ( TickType_t ) 0 );
  // }

  // if ( xWireSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  // {
  //   xWireSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
  //   if ( ( xWireSemaphore ) != NULL )
  //     xSemaphoreGive( xWireSemaphore );
  // }

  pinMode(INFO_LED, OUTPUT);


  
  // Now set up two Tasks to run independently.
  xTaskCreate(
    TaskFastPeriod
    ,  (const portCHAR *)"DR1"  // A name just for humans
    ,  256 // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 1 being the highest, and 4 being the lowest.
    ,  &xHandle1 );

  xTaskCreate(
    TaskTemperatureRead
    ,  (const portCHAR *) "TR1"
    ,  configMINIMAL_STACK_SIZE - 30  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  &xHandle2 );

  Serial.println("Test A");

  // xTaskCreate(
  //   TaskTimeRead
  //   ,  (const portCHAR *) "TR2"
  //   ,  configMINIMAL_STACK_SIZE - 20  // Stack size
  //   ,  NULL
  //   ,  1  // Priority
  //   ,  &xHandle3 );

  // Now the Task scheduler, which takes over control of scheduling individual Tasks, is automatically started.
}

void loop()
{
  tick++;
  digitalWrite(13, LOW);

  // set_sleep_mode( SLEEP_MODE_PWR_DOWN );
  // sleep_enable();
  // sleep_mode();

  // ZZzzZZzzZZzz
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

char buff[32];

void TaskFastPeriod( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  // char buff[32];
  pinMode(PIN_BTN_YLW, INPUT_PULLUP);
  int ut = 0;
  int buttonState = digitalRead(PIN_BTN_YLW);
  static int pwm_state = 0;

  for (;;) // A Task shall never return or exit.
  {
    Serial.println(millis());
    // read the input pin:
    buttonState = digitalRead(PIN_BTN_YLW);
    digitalWrite(INFO_LED, !buttonState);

    if(!buttonState)    // Apretat
      analogWrite(PIN_PWM, pwm_state++);

    // if ( xSemaphoreTake( xMySemaphore, ( TickType_t ) 0 ) == pdTRUE )
    // {
      // if ( xSemaphoreTake( xWireSemaphore, ( TickType_t ) 10 ) == pdTRUE ) {
        
        ut = now.unixtime();

        // Serial.println(buff);
        // oled.home();
        // oled.set2X();

        // // sprintf(buff, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
        // // oled.println(buff);

        // sprintf(buff, "%d.%d C", int(DHT.temperature*10)/10, int(DHT.temperature*10)%10);
        // oled.println(buff);

        // sprintf(buff, "%d.%d Hum.", int(DHT.humidity*10)/10, int(DHT.humidity*10)%10);
        // oled.println(buff);

        // oled.set1X();
        // // oled.println(dReadTime);
        // // oled.println(tick);
        // oled.println(pwm_state);
        // // oled.println(sleeping_time);

        // sprintf(buff, "%03d - %03d - %03d", 
        //   uxTaskGetStackHighWaterMark(xHandle1),
        //   uxTaskGetStackHighWaterMark(xHandle2),
        //   uxTaskGetStackHighWaterMark(xHandle3)
        // );
        // oled.println(buff);
        
        // xSemaphoreGive( xWireSemaphore );
      // }
    // }
    vTaskDelay(FREQ_100_MS / portTICK_PERIOD_MS);
    //vTaskDelay(1);
  }
}

void TaskTemperatureRead( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  TickType_t xLastWakeTime = xTaskGetTickCount ();
  TickType_t xFrequency = FREQ_500_MS / portTICK_PERIOD_MS ;
  
  for (;;)
  {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    digitalWrite(13, HIGH);
    // Serial.println(F("TaskTemperatureRead"));

    dReadTime = micros();
    
    // taskENTER_CRITICAL();
    while(DHT.read22(DHTPIN) != DHTLIB_OK)
      vTaskDelay(portTICK_PERIOD_MS);
    // taskEXIT_CRITICAL();
      
    dReadTime = micros() - dReadTime;
    digitalWrite(13, LOW);
      
    xSemaphoreGive( xMySemaphore ); // Now free or "Give" the Serial Port for others.
  }
}

// void TaskTimeRead( void *pvParameters __attribute__((unused)) )  // This is a Task.
// {
//   // TickType_t xLastWakeTime = xTaskGetTickCount ();
//   // TickType_t xFrequency = FREQ_500_MS / portTICK_PERIOD_MS ;
//   // bool state = true;

//   for (;;)
//   {
//     if ( xSemaphoreTake( xWireSemaphore, ( TickType_t ) 10 ) == pdTRUE ) {
//       now = rtc.now();
//       xSemaphoreGive( xWireSemaphore );
//     }
//     vTaskDelay(FREQ_500_MS / portTICK_PERIOD_MS);
//   }
// }
