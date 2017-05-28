// AVR Libraries
// #include <avr/power.h>
// #include <avr/sleep.h>

// Arduino libraries
#include <SoftwareSerial.h>

// Custom Arduino libraries
#include "../lib/project_cfg.h"
// #include "RTClib.h"         // https://github.com/adafruit/RTClib
// #include "IRremote.h"       // https://github.com/z3t0/Arduino-IRremote
// #include <Arduino_FreeRTOS.h>
// #include "semphr.h"  // add the FreeRTOS functions for Semaphores (or Flags).
// #include <MemoryFree.h>
// #include "dht.h"
// #include "LowPower.h"           // https://github.com/rocketscream/Low-Power
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPS++.h>

/* Custom defines */
#define SERIAL_BR 115200

/* Function prototypes */
void accel_calibration();
void accel_init();
void accel_tick();
void meansensors();
void calibration();
/***********************/

LiquidCrystal_I2C lcd(0x3f,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
MPU6050 mpu;

int buffersize=128;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
int16_t ax, ay, az, gx, gy, gz;
int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
// VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
// float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
struct ypr_degrees {
    float yaw;
    float pitch;
    float roll;
} ypr_deg;

SoftwareSerial gpsSerial(5, 4); // RX, TX
TinyGPSPlus gps;
#define UTC_OFFSET  (uint8_t)2

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ========================== End of Header ====================================
void setup()
{
  Serial.begin(SERIAL_BR);
  while(!Serial);

  gpsSerial.begin(9600);

  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  for(int p=0; p<PIN_COUNT; p++) {
    pinMode(p, INPUT);
    digitalWrite(p, LOW);
  }

  lcd.init();
  lcd.backlight();
  lcd.clear();

  // lcd.setBacklight(128);
  // lcd.print("Hello, world!");
  // delay(1000);
  // lcd.clear();

  pinMode(INFO_LED, OUTPUT);

  LED_ON;
  accel_init();
  LED_OFF;
}

// #define gpsSerial Serial
char activity[2] = {'|','-'};
uint8_t activity_counter = 0;
void loop()
{
    static long t333=millis();
    static long t1000=millis();
    static bool led_state = false;

    char buff[20];

    while(gpsSerial.available()) {
      // Serial.write(gpsSerial.read());
      gps.encode(gpsSerial.read());
    }

    if( (millis() - t1000) > 1000) {
      t1000=millis();

      sprintf(buff, "%d %d", (int)gps.passedChecksum(), (int)gps.failedChecksum());
      DEBUGLN(buff);

      led_state = !led_state;
      digitalWrite(INFO_LED, led_state);

      lcd.setCursor(1, 0);
      sprintf(buff, "Sat=%02d %02d:%02d:%02d", (int)gps.satellites.value(), gps.time.hour()+UTC_OFFSET, gps.time.minute(), gps.time.second());
      lcd.print(buff);

      // lcd.setCursor(0,1);
      // sprintf(buff, "Kmh=%03d Alt=%dm", (int)gps.speed.kmph(), (int)gps.altitude.meters());
      // lcd.print(buff);
    }

    // t=micros();
    // accel_tick();
    // int res = micros()-t;

    // if(res > 100) {
    //     DEBUG(millis()); DEBUG("\t"); DEBUG(res); DEBUGLN("us");
    // }

    if( (millis() - t333) > 333) {
        t333=millis();

        lcd.setCursor(0, 0);
        static uint32_t stat = gps.passedChecksum();
        uint32_t cur_stat = gps.passedChecksum();
        if(cur_stat > stat) {
          stat = cur_stat;
          lcd.setCursor(0, 0);
          lcd.write(activity[(activity_counter++)%2]);
        }

        if(false)
        {
            static int8_t old_angle = 0;
            // int8_t cur_angle = map(abs(ypr_deg.roll), 0, 80, 0, 8);
            int8_t cur_angle = map(ypr_deg.roll, -80, 80, -8, 8);
            // int8_t cur_angle = ypr_deg.roll/10;

            // if(ypr_deg.roll < 0)
            //   cur_angle = -cur_angle;

            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            lcd.setCursor(0,1);
            // sprintf(buff, "Kmh=%03d Alt=%dm", (int)gps.speed.kmph(), (int)gps.altitude.meters());
            // sprintf(buff, "%05d", az);
            // lcd.print(buff);

            if(old_angle != cur_angle) {
                // lcd.home();
                // lcd.clear();
                lcd.setCursor(0,1);
                memset(buff, " ", 16);
                lcd.print(buff);

                if(cur_angle > 0) {
                  /* Right */
                  lcd.setCursor(8,1);
                }
                else if(cur_angle < 0) {
                  /* Left */
                  lcd.setCursor(8+cur_angle,1);   // cur_angle is negative
                }
                else {
                  // lcd.setCursor(0,1);
                }
                memset(buff, '*', abs(cur_angle));
                lcd.print(buff);
                old_angle = cur_angle;
            }
        }
    }
}

void accel_tick()
{
    if(mpuInterrupt == true){
        mpuInterrupt = false;

        mpuIntStatus = mpu.getIntStatus();

        // get current FIFO count
        fifoCount = mpu.getFIFOCount();

        // check for overflow (this should never happen unless our code is too inefficient)
        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            // reset so we can continue cleanly
            mpu.resetFIFO();
            // DEBUGLN(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
        } else if (mpuIntStatus & 0x02) {
            // wait for correct available data length, should be a VERY short wait
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

            // read a packet from FIFO
            mpu.getFIFOBytes(fifoBuffer, packetSize);

            // track FIFO count here in case there is > 1 packet available
            // (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= packetSize;

            // display YPR
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

            ypr_deg.yaw = ypr[0] * 180/M_PI;
            ypr_deg.pitch = ypr[1] * 180/M_PI;
            ypr_deg.roll = ypr[2] * 180/M_PI;

            // DEBUG("ypr\t");
            // DEBUG(ypr[0] * 180/M_PI);
            // DEBUG("\t");
            // DEBUG(ypr[1] * 180/M_PI);
            // DEBUG("\t");
            // DEBUGLN(ypr[2] * 180/M_PI);
        }
    }
}

void accel_init()
{
	// initialize device
	DEBUGLN(F("Initializing I2C devices..."));
	mpu.initialize();
	pinMode(INTERRUPT_PIN, INPUT);

	// verify connection
	DEBUGLN(F("Testing device connections..."));
	DEBUGLN(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // mpu.setXAccelOffset(0);
    // mpu.setYAccelOffset(0);
    // mpu.setZAccelOffset(0);
    // mpu.setXGyroOffset(0);
    // mpu.setYGyroOffset(0);
    // mpu.setZGyroOffset(0);
    //
    // accel_calibration();

    // Sensor readings with offsets:   9       6       16388   1       -1      0
    // Your offsets:   -1114   -3106   1728    -10     -59     84

    // Sensor readings with offsets:   6       -5      16388   -1      1       2
    // Your offsets:   -1352   -3120   1709    -10     -59     85

    // Sensor readings with offsets:   -1      -13     16366   -1      1       0
    // Your offsets:   -1315   -3129   1707    -10     -59     84

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-10);
    mpu.setYGyroOffset(-59);
    mpu.setZGyroOffset(84);
    mpu.setZAccelOffset(1709); // 1688 factory default for my test chip

	// load and configure the DMP
    DEBUGLN(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        DEBUGLN(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        DEBUGLN(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        DEBUGLN(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        DEBUG(F("DMP Initialization failed (code "));
        DEBUG(devStatus);
        DEBUGLN(F(")"));
    }
}

void accel_calibration()
{
    bool calibrating_done = false;
    while(calibrating_done == false) {
        if (state==0){
          DEBUGLN("\nReading sensors for first time...");
          meansensors();
          state++;
          delay(1000);
        }

        if (state==1) {
          DEBUGLN("\nCalculating offsets...");
          calibration();
          state++;
          delay(1000);
        }

        if (state==2) {
          meansensors();
          DEBUGLN("\nFINISHED!");
          DEBUG("\nSensor readings with offsets:\t");
          DEBUG(mean_ax);
          DEBUG("\t");
          DEBUG(mean_ay);
          DEBUG("\t");
          DEBUG(mean_az);
          DEBUG("\t");
          DEBUG(mean_gx);
          DEBUG("\t");
          DEBUG(mean_gy);
          DEBUG("\t");
          DEBUGLN(mean_gz);
          DEBUG("Your offsets:\t");
          DEBUG(ax_offset);
          DEBUG("\t");
          DEBUG(ay_offset);
          DEBUG("\t");
          DEBUG(az_offset);
          DEBUG("\t");
          DEBUG(gx_offset);
          DEBUG("\t");
          DEBUG(gy_offset);
          DEBUG("\t");
          DEBUGLN(gz_offset);
          DEBUGLN("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
          DEBUGLN("Check that your sensor readings are close to 0 0 16384 0 0 0");
          DEBUGLN("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");

          calibrating_done = true;
        }
    }
}


///////////////////////////////////   FUNCTIONS   ////////////////////////////////////
void meansensors()
{
    long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

    while (i<(buffersize+101)) {
        // read raw accel/gyro measurements from device
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        if (i>100 && i<=(buffersize+100)) { //First 100 measures are discarded
            buff_ax=buff_ax+ax;
            buff_ay=buff_ay+ay;
            buff_az=buff_az+az;
            buff_gx=buff_gx+gx;
            buff_gy=buff_gy+gy;
            buff_gz=buff_gz+gz;
        }
        if (i==(buffersize+100)) {
            mean_ax=buff_ax/buffersize;
            mean_ay=buff_ay/buffersize;
            mean_az=buff_az/buffersize;
            mean_gx=buff_gx/buffersize;
            mean_gy=buff_gy/buffersize;
            mean_gz=buff_gz/buffersize;
        }
        i++;
        delay(2); //Needed so we don't get repeated measures
    }
}

void calibration()
{
    ax_offset=-mean_ax/8;
    ay_offset=-mean_ay/8;
    az_offset=(16384-mean_az)/8;

    gx_offset=-mean_gx/4;
    gy_offset=-mean_gy/4;
    gz_offset=-mean_gz/4;
    while (1) {
        int ready=0;
        mpu.setXAccelOffset(ax_offset);
        mpu.setYAccelOffset(ay_offset);
        mpu.setZAccelOffset(az_offset);

        mpu.setXGyroOffset(gx_offset);
        mpu.setYGyroOffset(gy_offset);
        mpu.setZGyroOffset(gz_offset);

        meansensors();
        DEBUGLN("...");

        if (abs(mean_ax)<=acel_deadzone) ready++;
        else ax_offset=ax_offset-mean_ax/acel_deadzone;

        if (abs(mean_ay)<=acel_deadzone) ready++;
        else ay_offset=ay_offset-mean_ay/acel_deadzone;

        if (abs(16384-mean_az)<=acel_deadzone) ready++;
        else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

        if (abs(mean_gx)<=giro_deadzone) ready++;
        else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

        if (abs(mean_gy)<=giro_deadzone) ready++;
        else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

        if (abs(mean_gz)<=giro_deadzone) ready++;
        else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

        if (ready==6) break;
    }
}
