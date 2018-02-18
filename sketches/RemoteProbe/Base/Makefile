#C:/Program Files (x86)/Arduino/hardware/arduino/avr/cores/arduino
#C:/Program Files (x86)/Arduino/hardware/tools/avr/avr/include
#C:/Program Files (x86)/Arduino/hardware/arduino/avr/libraries/SPI/src/
#C:/Program Files (x86)/Arduino/hardware/arduino/avr/libraries/Wire/src/
#C:/Program Files (x86)/Arduino/hardware/arduino/avr/libraries/SoftwareSerial/src/

#defines __AVR__ __AVR_ATmega328P__

PIO=pio

all:
	$(PIO) run
	
upload:
	$(PIO) run --target upload

clean:
	rm -f firmware.map
	$(PIO) run --target clean
	
monitor:
	$(PIO) device monitor
