#ifndef OLED_H
#define OLED_H

#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
extern SSD1306AsciiAvrI2c oled;

#define I2C_ADDRESS 0x3C



typedef enum t_oled_state { OLED_ON = 0, OLED_OFF };
static t_oled_state oled_state;

int oled_enable_pin;

void InitOled(int);
void WakeUpOled();
void SleepOled();

typedef struct {
    float temperature;
    float humidity;
} t_main_screen_data;
t_main_screen_data main_screen_data;
void RenderMainScreen();

#endif  /* OLED_H */
