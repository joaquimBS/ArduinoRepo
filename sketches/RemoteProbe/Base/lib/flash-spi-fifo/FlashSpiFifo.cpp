#include "FlashSpiFifo.h"

SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)

FlashSpiFifo::FlashSpiFifo(void)
{}

void FlashSpiFifo::init(void)
{
    if (flash.initialize()) {
        Serial.print("DeviceID: ");
        Serial.println(flash.readDeviceId(), HEX);
        // flash.blockErase4K(0);
        flash.sleep();
    }
    LED_ON;
}

void FlashSpiFifo::push(t_fifo_data element)
{
  static int value = 0x30;
  static int idx = 0;
  flash.writeByte(idx, value++);

  // int rd_val = flash.readByte(idx);
  // Serial.println(rd_val);

  idx++;
}

// t_fifo_data FlashSpiFifo::pop()
// {
//
//   return (t_fifo_data)0;
// }
