#ifndef FlashSpiFifo_h
#define FlashSpiFifo_h

#include "Arduino.h"
#include <SPIFlash.h>         //get it here: https://www.github.com/lowpowerlab/spiflash
#include "../project_cfg.h"

typedef struct {
  int temperature;
  unsigned int humidity;
  int sound_level;
  unsigned long long timestamp;
} t_fifo_data;

class FlashSpiFifo
{
  public:
    FlashSpiFifo(void);
    void init();
    void push(t_fifo_data element);
    // t_fifo_data pop();

  private:
    unsigned long num_elements_;
    unsigned int element_size_;
    unsigned long next_element_pos_;
};

#endif /* FlashSpiFifo */
