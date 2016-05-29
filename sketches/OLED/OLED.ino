#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display;

void setup() {
  // put your setup code here, to run once:

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();

  delay(2000);

  display.clearDisplay();


  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Hello, world!");

  display.display();

}

void loop() {
  // put your main code here, to run repeatedly:

}
