PIO=pio

OBJS=   \
        src/Thermostat.cpp \
        src/Thermostat.h

all:
	$(PIO) run --target upload
	
build:
	$(PIO) run

clean:
	$(PIO) run --target clean
	
monitor:
	$(PIO) device monitor
