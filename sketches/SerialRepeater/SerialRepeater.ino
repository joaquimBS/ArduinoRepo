
#include <SoftwareSerial.h>
// software serial #1: RX TX
SoftwareSerial portOne(4, 5);

void setup() {
  Serial.begin(19200);
  while (!Serial);
  
  portOne.begin(19200);

  // Serial.println("Running...");
}

void loop() {
  String inBuffer;
  
  if(portOne.available()) {
    char in = portOne.read();
    Serial.write(in);
  }

  if(Serial.available()) {
    char in = Serial.read();
    portOne.write(in);
  }
}
