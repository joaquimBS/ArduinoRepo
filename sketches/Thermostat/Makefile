PIO=pio

OBJS=   \
        src/Thermostat.cpp \
        src/Thermostat.h

all:
	$(PIO) run
	
upload:
	$(PIO) run --target upload

clean:
	$(PIO) run --target clean
	
monitor:
	$(PIO) device monitor
