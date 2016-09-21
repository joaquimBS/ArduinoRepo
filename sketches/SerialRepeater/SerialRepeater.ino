
#include <SoftwareSerial.h>
#include <DHTlib/dht.h>

#define PIN_BTN_2   2   // INT0
#define PIN_BTN_3   3   // INT1
#define DHTPIN      4

dht DHT;

// software serial #1: RX TX
SoftwareSerial portOne(6, 5); // RX, TX

long elapsed = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  
  portOne.begin(9600);

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  pinMode(PIN_BTN_2, INPUT_PULLUP);

  elapsed = millis();

  portOne.print(F("AT+RST\r\n"));

  Serial.println("Running...");
}

int step=0;
bool enabled = false;

// https://data.sparkfun.com/streams/o87Zlv6Z8AF5o0z673Wl
// yz4YqkJYzMiBRdjvxDXJ

long t1000 = millis();
char buff[32];

void loop() {
  static int val = 0;

  if((millis() - t1000) > 1000) {
    t1000 = millis();

    while(DHT.read(DHTPIN) != DHTLIB_OK);
  }

  if((millis() - elapsed) > 250) {

    if(!digitalRead(PIN_BTN_2)) {
      enabled = true;
      step=0;

      digitalWrite(13, 1);
      delay(250);
      digitalWrite(13, 0);

      elapsed = millis();
    }
  }

  if(enabled && (millis() - elapsed) > val) {
    elapsed = millis();

    switch(step) {
      case 0:
        portOne.print(F("AT+CIPMUX=1\r\n"));
        val = 500;
        break;
      case 1:
        portOne.print(F("AT+CIPSTART=4,\"TCP\",\"data.sparkfun.com\",80\r\n"));
        val = 2000;
        break;
      case 2:
        portOne.print(F("AT+CIPSEND=4,141\r\n"));
        val = 500;
        break;
      case 3:
        portOne.print(F("GET /input/o87Zlv6Z8AF5o0z673Wl?private_key=yz4YqkJYzMiBRdjvxDXJ"));
        
        sprintf(buff, "&temp=%02d.%d", int(DHT.temperature*10)/10, int(DHT.temperature*10)%10);
        portOne.print(buff);

        sprintf(buff, "&hum=%02d.%d", int(DHT.humidity*10)/10, int(DHT.humidity*10)%10);
        portOne.print(buff);

        portOne.print(" HTTP/1.1\r\n");
        portOne.print(F("Host: data.sparkfun.com\r\n"));
        portOne.print(F("Connection: close\r\n"));
        portOne.print(F("\r\n"));
        portOne.print(F("\r\n"));

        break;
      case 4:
        enabled=false;
        step = -1;
        break;
    }
    step++;
  }
  
  while(portOne.available()) {
    char in = portOne.read();
    Serial.write(in);
  }

  while(Serial.available()) {
    char in = Serial.read();
    portOne.write(in);
  }
}
