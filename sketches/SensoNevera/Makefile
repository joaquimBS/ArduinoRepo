#C:/Program Files (x86)/Arduino/hardware/arduino/avr/cores/arduino
#C:/Program Files (x86)/Arduino/hardware/tools/avr/avr/include
#C:/Program Files (x86)/Arduino/hardware/arduino/avr/libraries/SPI/src/
#C:/Program Files (x86)/Arduino/hardware/arduino/avr/libraries/Wire/src/
#C:/Program Files (x86)/Arduino/hardware/arduino/avr/libraries/SoftwareSerial/src/

#defines __AVR__ __AVR_ATmega328P__

ENV=uno
PIO=pio

all:
	$(PIO) run --environment $(ENV)
	
upload:
	$(PIO) run --environment $(ENV) --target upload

clean:
	rm -f firmware.map
	$(PIO) run --environment $(ENV) --target clean
	
monitor:
	$(PIO) device monitor -b 115200
