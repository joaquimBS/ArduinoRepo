
#include "freeRTOS.h"

// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only only one Task is accessing this resource at any time.
// SemaphoreHandle_t xSerialSemaphore;

unsigned long tick = 0;

// define two Tasks for DigitalRead & AnalogRead
void TaskDigitalRead( void *pvParameters );
void TaskTemperatureRead( void *pvParameters );

DHT dht(DHTPIN, DHTTYPE);
SSD1306AsciiAvrI2c oled;

// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);

  // oled.begin(&Adafruit128x64, I2C_ADDRESS);
  // oled.setFont(System5x7);
  // oled.clear();

  dht.begin();

  


  // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
  // because it is sharing a resource, such as the Serial port.
  // Semaphores should only be used whilst the scheduler is running, but we can set it up here.
  // if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  // {
  //   xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
  //   if ( ( xSerialSemaphore ) != NULL )
  //     xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  // }

  pinMode(INFO_LED, OUTPUT);
  

  // Now set up two Tasks to run independently.
  // xTaskCreate(
  //   TaskDigitalRead
  //   ,  (const portCHAR *)"DR1"  // A name just for humans
  //   ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
  //   ,  NULL
  //   ,  1  // Priority, with 1 being the highest, and 4 being the lowest.
  //   ,  NULL );

  xTaskCreate(
    TaskTemperatureRead
    ,  (const portCHAR *) "TR1"
    ,  128  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL );

  // Now the Task scheduler, which takes over control of scheduling individual Tasks, is automatically started.
}


void loop()
{
  // Empty. Things are done in Tasks.
  // tick++;
  // digitalWrite(13, LOW);
  // set_sleep_mode( SLEEP_MODE_PWR_DOWN );
  // sleep_enable();
  // sleep_mode();
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/


// void TaskDigitalRead( void *pvParameters __attribute__((unused)) )  // This is a Task.
// {
//   /*
//     DigitalReadSerial
//     Reads a digital input on pin 2, prints the result to the serial monitor

//     This example code is in the public domain.
//   */

//   // digital pin 2 has a pushbutton attached to it. Give it a name:
//   // make the pushbutton's pin an input:
//   pinMode(PIN_BTN_YLW, INPUT_PULLUP);

//   for (;;) // A Task shall never return or exit.
//   {
//     // read the input pin:
//     int buttonState = digitalRead(PIN_BTN_YLW);
//     digitalWrite(INFO_LED, !buttonState);

//     // if(!buttonState) {
//     // Serial.print(buttonState);
//     vTaskDelay(50 / portTICK_PERIOD_MS);

//     // }

//     // See if we can obtain or "Take" the Serial Semaphore.
//     // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
//     // if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
//     // {
//       // We were able to obtain or "Take" the semaphore and can now access the shared resource.
//       // We want to have the Serial Port for us alone, as it takes some time to print,
//       // so we don't want it getting stolen during the middle of a conversion.
//       // print out the state of the button:
      

//       // xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
//     // }

//     // vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
//   }
// }

#define FREQ_MS  2000
void TaskTemperatureRead( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  TickType_t xLastWakeTime = xTaskGetTickCount ();
  TickType_t xFrequency = FREQ_MS / portTICK_PERIOD_MS ;
 
  for (;;)
  {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    // digitalWrite(13, HIGH);

    // taskENTER_CRITICAL();

    float t = dht.readTemperature();
    Serial.println(t);
    
    // oled.home();
    // oled.println(dht.readTemperature());
    // oled.println(tick);
    // taskEXIT_CRITICAL();

    // delay(50);
    // old_millis = millis();

    
    

    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    // if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    // {
      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      // print out the value you read:
      // Serial.println(millis() - old_millis);
      

      // Serial.println(xFrequency);
      // Serial.println(portTICK_PERIOD_MS);
      // Serial.println(xLastWakeTime);
      
      // Serial.println(sensorValue);

      // xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    // }

    // diff_millis = FREQ_MS - (millis() - old_millis);
    // Serial.println(diff_millis);
    // xFrequency = (FREQ_MS + diff_millis) / portTICK_PERIOD_MS ;
    // vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}



// /******************************************************************************
// FreeRTOS timed blink example

// Byron Jacquot @ SparkFun Electronics>

// Porting Marshall Taylor's out-of-synch blinker example
// to an RTOS, to show off basic features.

// Resources:
// Requires Bill Greiman's port of Free RTOS to Arduino
// https://github.com/greiman/FreeRTOS-Arduino

// Development environment specifics:
// Arduino 1.6.5
// SparkFun RebBoard with additional LED on pin 12.

// This code is released under the [MIT License](http://opensource.org/licenses/MIT).
// Distributed as-is; no warranty is given.
// ******************************************************************************/

// #include <Arduino_FreeRTOS.h>

// // define two tasks for Blink & AnalogRead
// void TaskBlink( void *pvParameters );
// void TaskAnalogRead( void *pvParameters );

// void setup() 
// {

//   // Now set up two tasks to run independently.
//   xTaskCreate(
//     TaskBlink
//     ,  (const portCHAR *)"Blink"   // A name just for humans
//     ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
//     ,  NULL
//     ,  2  // Priority, with 1 being the highest, and 4 being the lowest.
//     ,  NULL );

//   xTaskCreate(
//     TaskAnalogRead
//     ,  (const portCHAR *) "AnalogRead"
//     ,  128  // Stack size
//     ,  NULL
//     ,  1  // Priority
//     ,  NULL );

//   // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
// }

// //------------------------------------------------------------------------------
// // WARNING idle loop has a very small stack (configMINIMAL_STACK_SIZE)
// // loop must never block
// void loop() 
// {
//   // Empty. Things are done in Tasks.
// }

// /*--------------------------------------------------*/
// /*---------------------- Tasks ---------------------*/
// /*--------------------------------------------------*/

// void TaskBlink(void *pvParameters)  // This is a task.
// {
//   (void) pvParameters;

//   // initialize digital pin 13 as an output.
//   pinMode(13, OUTPUT);

//   for (;;) // A Task shall never return or exit.
//   {
//     digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
//     vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
//     digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
//     vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
//   }
// }

// void TaskAnalogRead(void *pvParameters)  // This is a task.
// {
//   (void) pvParameters;

//   // initialize serial communication at 9600 bits per second:
//   Serial.begin(9600);

//   for (;;)
//   {
//     // read the input on analog pin 0:
//     int sensorValue = analogRead(A0);
//     // print out the value you read:
//     Serial.println(sensorValue);
//     vTaskDelay(1);  // one tick delay (30ms) in between reads for stability
//   }
// }

///////////////////////////////////////////////////////////////////////////////////