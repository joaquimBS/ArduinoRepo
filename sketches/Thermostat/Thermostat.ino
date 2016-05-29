#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>

#include "Thermostat.h"

dht DHT;
Sleep sleep;
RTC_DS1307 rtc;

class LCD_Manager {

public:
	LCD_Manager() {}

	void renderMainScreen(unsigned int currentT, unsigned int objectiveT, bool isHeatOn)
	{
		char buff[17];
		sprintf(buff, "Temp: %d.%d ", currentT/10, currentT%10);
		strcat(buff, isHeatOn ? "ON" : "OFF");
		Serial.println(buff);

		sprintf(buff, "Obje: %d.%d ", objectiveT/10, objectiveT%10);
		Serial.println(buff);
	}
};

LCD_Manager lcd;

void setup() {
	Serial.begin(115200);
	while(!Serial);

	pinMode(PIN_BTN_MORE, 	INPUT);

	pinMode(13,OUTPUT);
	digitalWrite(13, LOW);

	pinMode(4,OUTPUT);
	digitalWrite(4, LOW);

	delay(1000);

	for(int i=0; i<20; i++) {

		_isHumanInteraction = false;

		digitalWrite(4, HIGH);
		digitalWrite(13, HIGH);
		delay(1000);
		digitalWrite(4, LOW);
		digitalWrite(13, LOW);


		goToSleep(120000);


		delay(500);
		Serial.println("Back!");
	}





	pinMode(RELE_PIN, 	OUTPUT);
	pinMode(INFO_LED, 	OUTPUT);
	pinMode(DHT22_VCC, 	OUTPUT);
	pinMode(RTC_VCC, 	OUTPUT);
	pinMode(SD_VCC, 	OUTPUT);

	pinMode(PIN_BTN_MORE, 	INPUT);
	pinMode(PIN_BTN_LESS, 	INPUT);

	_loadSettingsFromEeprom();

	digitalWrite(INFO_LED, 	LOW);
	digitalWrite(DHT22_VCC, HIGH);
	digitalWrite(RTC_VCC, LOW);
	digitalWrite(SD_VCC, LOW);

	// pinMode(2,INPUT_PULLUP);
	// attachInterrupt(digitalPinToInterrupt(2), _readTempHum, FALLING);

	//////////////////////////////

	_initRTC();
	_initSD();
	_readTempHum();

	_timerPeriod100 = millis();
	_timerPeriod1000 = millis();
	_timerToSleep = millis();

	lcd.renderMainScreen(_tempActual, _thermoSettings.temp_objective, isHeatOn);
}

bool rele_state = false;
void loop()
{
	// float vBat = analogRead(A0);
	// vBat = (vBat*5.0) / 1023.0;
	// Serial.println(vBat);

	_readButtons();
	
	if((millis() - _timerPeriod100) > 100) {
		_timerPeriod100 = millis();

	}

	if((millis() - _timerPeriod1000) > 1000) {
		_timerPeriod1000 = millis();
		_dataLogger();		
	}

	if((millis() - _timerToSleep) > TIMEOUT_TO_HEAT) {
		if(_wasButtonPressed) {
			_wasButtonPressed = false;
			if(isHeatRequired())
				TurnHeatON()
			else
				TurnHeatOFF()

			lcd.renderMainScreen(_tempActual, _thermoSettings.temp_objective, isHeatOn);
		}
	}

	if((millis() - _timerToSleep) > TIMEOUT_TO_SLEEP) {
		_isHumanInteraction = false;

		while(!_isHumanInteraction) {
			_inSleepRoutine();
			goToSleep(SLEEP_TIME_MS);
			// ZZzzZZzzZZ
		}
		_postSleepRoutine();
		_timerToSleep = millis();
	}
}

static unsigned long debounce = 0;
void _readButtons()
{
	if((millis() - debounce) < 250) return;

	if(digitalRead(PIN_BTN_MORE)) {
		_timerToSleep = millis();
		_lastButtonPressed = BTN_MORE;
		_wasButtonPressed = true;
		debounce = millis();
	}
		
	else if(digitalRead(PIN_BTN_LESS)) {
		_timerToSleep = millis();
		_lastButtonPressed = BTN_LESS;
		_wasButtonPressed = true;
		debounce = millis();
	}
	else
		_lastButtonPressed = BTN_NONE;


	if(_lastButtonPressed != BTN_NONE)
		_processButtonPress();
}

void _processButtonPress()
{
	if(_lastButtonPressed == BTN_MORE) {
		_thermoSettings.temp_objective += 5;	// half a degree;
	}
	else if(_lastButtonPressed == BTN_LESS) {
		_thermoSettings.temp_objective -= 5;	// half a degree;
	}

	_storedSettingsToEeprom();

	lcd.renderMainScreen(_tempActual, _thermoSettings.temp_objective, isHeatOn);
}

void _inSleepRoutine()
{
	// pinMode(RTC_VCC, OUTPUT);
	digitalWrite(SD_VCC, LOW);
	digitalWrite(RTC_VCC, LOW);

	_readTempHum();

	rtc.begin();

	_printDateTime();

	if(isHeatRequired())
		TurnHeatON()
	else
		TurnHeatOFF()

	Serial.print(F("\tHeat: "));
	Serial.println(isHeatOn ? "ON" : "OFF");

	// digitalWrite(RTC_VCC, HIGH);
	// digitalWrite(SD_VCC, HIGH);
	// pinMode(RTC_VCC, INPUT);
}

void _postSleepRoutine()
{
	lcd.renderMainScreen(_tempActual, _thermoSettings.temp_objective, isHeatOn);
	debounce = millis();
}

bool isHeatRequired()
{
	// static bool ret = true;

	if(_tempActual >= _thermoSettings.temp_objective)
		isHeatOn = false;
	else
		isHeatOn = true;

	return isHeatOn;
}

void _readTempHum()
{
	unsigned long ms = millis();
	digitalWrite(DHT22_VCC,LOW);
	digitalWrite(INFO_LED, HIGH);

	isTempHumOK = true;

	while(DHT.read22(DHT22_PIN) != DHTLIB_OK) {
		if((millis() - ms) > 3000) {
			isTempHumOK = false;
			break;
		}
		delay(150);
	}

	if(isTempHumOK)
	{
		// char buffer[32];

		_tempActual = int(DHT.temperature*10);
		// sprintf(buffer, "_readTempHum()\t %d ms\t%d\t%d", (int)(millis() - ms), _tempActual, int(DHT.humidity*10));
		// Serial.println(buffer);
		// delay(25);
	}

	digitalWrite(INFO_LED,LOW);
	digitalWrite(DHT22_VCC, HIGH);
}

void _dataLogger()
{
	if(!isSDInit) return;

	int ms = millis();
	digitalWrite(INFO_LED,HIGH);

	char logline[64];
	DateTime now = rtc.now();

	sprintf(logline, "%02d%02d%02d;", now.hour(), now.minute(), now.second());

	// TEMPERATURE AND HUMIDITY
	{
		char tempHum[11];
		if(isTempHumOK)
			sprintf(tempHum, "%d;%d;", int(DHT.temperature*10), int(DHT.humidity*10));
		else
			strcpy(tempHum, "0;0;");

		strcat(logline, tempHum);
	}
	
	char fileName[32];
	sprintf(fileName, "%d%02d%02d.csv", now.year(), now.month(), now.day());
	File logFile = SD.open(fileName, FILE_WRITE);
	if(logFile) {
		logFile.seek(logFile.size());
		logFile.println(logline);
		logFile.close();
	}

	{
		char buffer[128];
		sprintf(buffer, "_dataLogger()\t %d ms\t", (int)(millis() - ms));
		strcat(buffer, logline);
		Serial.println(buffer);
		delay(25);
	}

	digitalWrite(INFO_LED,LOW);
}

void pin2Interrupt(void)
{
	_isHumanInteraction = true;
	detachInterrupt(digitalPinToInterrupt(PIN_BTN_MORE));
}

void goToSleep(unsigned long ms)
{
	/*
	Shut down LCD, SD, or any other external device.
	*/
	digitalWrite(RTC_VCC, HIGH);
	digitalWrite(SD_VCC, HIGH);

	Serial.println(F("\tZZzzZZzzZZ"));
	delay(50);

	EIFR = 0x01;
	attachInterrupt(digitalPinToInterrupt(PIN_BTN_MORE), pin2Interrupt, RISING);
	
	sleep.pwrDownMode();
	// sleep.idleMode(); //set sleep mode
	
	// sleep.sleepInterrupt(0,CHANGE); //(interrupt Number, interrupt State)
	sleep.sleepDelay(ms, _isHumanInteraction);
	/* ZZzzZZzz */
}

void _initSD()
{
	isSDInit = false;

	if(!SD.begin(SD_CS)) {
		Serial.println("SD\tFail!");
		digitalWrite(INFO_LED, HIGH);
		delay(2000);
		digitalWrite(INFO_LED, LOW);
	}
	else {
		DateTime now = rtc.now();
		char fileName[32];
		sprintf(fileName, "%d%02d%02d.csv", now.year(), now.month(), now.day());
		if(!SD.exists(fileName)) {
			File logFile = SD.open(fileName, FILE_WRITE);
			if(logFile) {
				logFile.println("timestamp;temp;humidity;");
				logFile.close();
			}
		}
		isSDInit = true;
		Serial.println("SD\tOK!");
	}
}

void _initRTC()
{
	isRTCInit = false;

	if(rtc.begin() && rtc.isrunning()) {
		isRTCInit = true;
		Serial.println("RTC\tOK!");
		_printDateTime();
	}
	else {
		Serial.println("RTC\tFail!");
		digitalWrite(INFO_LED, HIGH);
		delay(2000);
		digitalWrite(INFO_LED, LOW);
	}
	// rtc.adjust(DateTime(1448115499));
	// rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

void _blinkInfoLed(unsigned int times)
{
	digitalWrite(INFO_LED, LOW);

	for(unsigned int i=0; i<times; ++i) {
		digitalWrite(INFO_LED, HIGH);
		delay(150);
		digitalWrite(INFO_LED, LOW);
		delay(150);
	}

	digitalWrite(INFO_LED, LOW);
}

void _printDateTime()
{
	if(isRTCInit) {
		DateTime now = rtc.now();

		Serial.print(now.year(), DEC);
		Serial.print('/');
		Serial.print(now.month(), DEC);
		Serial.print('/');
		Serial.print(now.day(), DEC);
		Serial.print(' ');
		Serial.print(now.hour(), DEC);
		Serial.print(':');
		Serial.print(now.minute(), DEC);
		Serial.print(':');
		Serial.print(now.second(), DEC);
		Serial.println();

		Serial.println(now.unixtime());

		delay(100);
	}
}

void _loadSettingsFromEeprom()
{
	byte b0 = EEPROM.read(0);
	byte b1 = EEPROM.read(1);

	if((b0 == CTRL_B0) && (b1 == CTRL_B1))
		EEPROM.get(2, _thermoSettings);
	else {
		_thermoSettings.temp_objective = DEFAULT_OBJECTIVE;

		// More settings to come
		// ...

		EEPROM.write(0, CTRL_B0);
		EEPROM.write(1, CTRL_B1);
		EEPROM.put(2, _thermoSettings);

		_loadSettingsFromEeprom();
	}

	_thermoSettings.toSerial();
}

void _storedSettingsToEeprom()
{
	EEPROM.write(0, CTRL_B0);
	EEPROM.write(1, CTRL_B1);
	EEPROM.put(2, _thermoSettings);
}