#include "Oled.h"

void InitOled(int oled_en_pin)
{
    if(oled_state == OLED_ON)
        return;

    oled_enable_pin = oled_en_pin;
    WakeUpOled();
}

void WakeUpOled()
{
    digitalWrite(oled_enable_pin, HIGH);
    delay(150);
    oled.begin(&Adafruit128x64, I2C_ADDRESS);
    oled.setFont(Stang5x7);
    oled.set2X();

    oled_state = OLED_ON;
}

void SleepOled()
{
    digitalWrite(oled_enable_pin, HIGH);
    oled_state = OLED_OFF;
}

void RenderMainScreen()
{
	char buff[16];

    oled.home();

    sprintf(buff, "%d.%d C",int(main_screen_data.temperature*10)/10,
                            int(main_screen_data.temperature*10)%10);
    /* Line 1 */
    oled.println(buff);

	sprintf(buff, "%d.%d Hum.", int(main_screen_data.humidity*10)/10,
                                int(main_screen_data.humidity*10)%10);
    /* Line 2 */
    oled.println(buff);

    /* Line 3 */
	oled.println(millis());
}
