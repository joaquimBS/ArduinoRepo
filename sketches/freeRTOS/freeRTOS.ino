
#include "freeRTOS.h"

// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only only one Task is accessing this resource at any time.
SemaphoreHandle_t xMySemaphore;
SemaphoreHandle_t xWireSemaphore;

unsigned long tick = 0;
unsigned long dReadTime = 0;

// define two Tasks for DigitalRead & AnalogRead
void TaskFastPeriod( void *pvParameters );
void TaskTemperatureRead( void *pvParameters );
void TaskTimeRead( void *pvParameters );

#define FREQ_1000_MS  1000
#define FREQ_500_MS   500
#define FREQ_100_MS   100

dht DHT;
SSD1306AsciiAvrI2c oled;
RTC_DS1307 rtc;
DateTime now;

void InitRtc(int timeout_ms)
{
  bool isRTCInit = false;

  rtc.begin();

  unsigned long t0 = millis();
  

  while(!isRTCInit && ((millis() - t0) < timeout_ms) ) {
    if(rtc.isrunning())
      isRTCInit = true;
    else {
      delay(25);
      Serial.println(millis());
    }
  }

  if(isRTCInit) {
    Serial.println("RTC\tOK!");
    _printDateTime();
  }
  else {
    Serial.println("RTC\tFail!");
  }

  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}


void _printDateTime()
{
  // if(isRTCInit) {
    now = rtc.now();

    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();

    Serial.println(now.unixtime());

    // delay(100);
  // }
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
  Serial.begin(115200);

  InitOled();
  // delay(100);
  // InitRtc(100);

  // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
  // because it is sharing a resource, such as the Serial port.
  // Semaphores should only be used whilst the scheduler is running, but we can set it up here.
  if ( xMySemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xMySemaphore = xSemaphoreCreateBinary();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xMySemaphore ) != NULL )
      xSemaphoreTake( xMySemaphore, ( TickType_t ) 0 );
  }

  if ( xWireSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xWireSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xWireSemaphore ) != NULL )
      xSemaphoreGive( xWireSemaphore );
  }

  pinMode(INFO_LED, OUTPUT);
  

  // Now set up two Tasks to run independently.
  xTaskCreate(
    TaskFastPeriod
    ,  (const portCHAR *)"DR1"  // A name just for humans
    ,  256  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 1 being the highest, and 4 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskTemperatureRead
    ,  (const portCHAR *) "TR1"
    ,  101   // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

  xTaskCreate(
    TaskTimeRead
    ,  (const portCHAR *) "TR2"
    ,  100  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

  // Now the Task scheduler, which takes over control of scheduling individual Tasks, is automatically started.
}


void loop()
{
  tick++;
  digitalWrite(13, LOW);

  set_sleep_mode( SLEEP_MODE_PWR_DOWN );
  sleep_enable();
  sleep_mode();
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

  for (;;) // A Task shall never return or exit.
  {
    // read the input pin:
    buttonState = digitalRead(PIN_BTN_YLW);
    digitalWrite(INFO_LED, !buttonState);

    if ( xSemaphoreTake( xMySemaphore, ( TickType_t ) 0 ) == pdTRUE )
    {
      if ( xSemaphoreTake( xWireSemaphore, ( TickType_t ) 10 ) == pdTRUE ) {
        
        ut = now.unixtime();

        // Serial.println(buff);
        oled.home();
        oled.set2X();

        sprintf(buff, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
        oled.println(buff);

        sprintf(buff, "%d.%d C", int(DHT.temperature*10)/10, int(DHT.temperature*10)%10);
        oled.println(buff);

        sprintf(buff, "%d.%d Hum.", int(DHT.humidity*10)/10, int(DHT.humidity*10)%10);
        oled.println(buff);

        oled.set1X();
        oled.println(dReadTime);
        oled.println(tick);

        xSemaphoreGive( xWireSemaphore );
      }
    }
    vTaskDelay(FREQ_100_MS / portTICK_PERIOD_MS);
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

void TaskTimeRead( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  TickType_t xLastWakeTime = xTaskGetTickCount ();
  TickType_t xFrequency = FREQ_500_MS / portTICK_PERIOD_MS ;
  bool state = true;

  for (;;)
  {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    if ( xSemaphoreTake( xWireSemaphore, ( TickType_t ) 10 ) == pdTRUE ) {
      now = rtc.now();
      xSemaphoreGive( xWireSemaphore );
    }
  }
}
