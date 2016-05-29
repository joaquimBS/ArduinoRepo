#include "ProtoTest1.h"

#define BUZ	2

#define RED 8
#define GRE 9
#define BLU 10

int r=0;
int g=0;
int b=0;
unsigned long _ledFlag;

unsigned long _morseFlag;
unsigned int iCharacter = 0;
unsigned char vCharacter = '0';

//Pin connected to ST_CP of 74HC595
int latchPin = 8;
//Pin connected to SH_CP of 74HC595
int clockPin = 12;
////Pin connected to DS of 74HC595
int dataPin = 11;

unsigned long _7segNums[10] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x67};

float tempC;
int reading;
int tempPin = 0;

void setup() {
	// put your setup code here, to run once:
	/*
	pinMode(RED, OUTPUT);
	pinMode(GRE, OUTPUT);
	pinMode(BLU, OUTPUT);

	digitalWrite(RED, 0);
	digitalWrite(GRE, 0);
	digitalWrite(BLU, 0);

	_ledFlag = millis();

	pinMode(BUZ, OUTPUT);
	digitalWrite(BUZ, 0);
	_morseFlag = millis();
	*/

	pinMode(2, OUTPUT);
	delay(50);
	digitalWrite(2, HIGH);

	Serial.begin(115200);
	while(!Serial);

	analogReference(INTERNAL);
	pinMode(A0, INPUT);

	pinMode(latchPin, OUTPUT);
	pinMode(clockPin, OUTPUT);
	pinMode(dataPin, OUTPUT);

	digitalWrite(latchPin, LOW);
	shiftOut(dataPin, clockPin, MSBFIRST, 0);
	digitalWrite(latchPin, HIGH);

	digitalWrite(2, LOW);

	delay(500);

}

void loop() {
	for (unsigned int ntd = 0; ntd < 20; ntd++) {
		// take the latchPin low so 
		// the LEDs don't change while you're sending in bits:

		//Serial.println(_7segNums[ntd], HEX);

		digitalWrite(latchPin, LOW);
		// shift out the bits:
		if(ntd<10)
			shiftOut(dataPin, clockPin, MSBFIRST, _7segNums[ntd]);
		else {
			shiftOut(dataPin, clockPin, MSBFIRST, (_7segNums[ntd-10]) | 0x80);
		}
		//shiftOut(dataPin, clockPin, MSBFIRST, ntd);

		//take the latch pin high so the LEDs will light up:
		digitalWrite(latchPin, HIGH);

		reading = analogRead(tempPin);
		tempC = reading / 9.31;
		Serial.println(tempC);

		// pause before next value:
		delay(500);
	}

	//rgbLED();

	//morseCode();
}


void morseCode()
{
	if(!_morseFlag) return;

	vCharacter = strText[iCharacter];

	vCharacter -= 'A';

	_txMorseChar(vCharacter);
}

void _txMorseChar(unsigned char c) 
{
	/*
	for(int i=0; i<sizeof(MorseAlphabet[c]); i++) {
		(MorseAlphabet[c]) ? dah() : dih();
	}
	*/
}

void rgbLED()
{
	if((millis() - _ledFlag) < 1000) return;

	_ledFlag = millis();

	r++;
	if(r==2) {
		r=0;
		g++;
		if(g==2) {
			g=0;
			b++;
			if(b==2) b=0;
		}
	}
	digitalWrite(RED, r);
	digitalWrite(GRE, g);
	digitalWrite(BLU, b);
}

void dah()
{
	digitalWrite(BUZ, 1);
	delay(300);
	digitalWrite(BUZ, 0);
}

void dih()
{
	digitalWrite(BUZ, 1);
	delay(100);
	digitalWrite(BUZ, 0);
}