
void setup() {
	Serial.begin(115200);
	pinMode(13, OUTPUT);

	randomSeed(analogRead(0));
}

void loop() {
	unsigned long t0 = millis();

	for(int i=0; i<1000; i++)
		random(0, 16000);
	
	Serial.println(millis()-t0);
	delay(500);
}
