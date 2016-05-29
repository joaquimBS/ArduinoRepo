static const unsigned int ITERATIONS = 65535;
unsigned int t0;
unsigned int i;
unsigned int j;
unsigned long total;
boolean val = false;
boolean val2 = false;
char buffer[32];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(13, OUTPUT);

}

void loop() {
	t0 = millis();
	i=0;

	while(++i != ITERATIONS) {
		j=0;
		while(++j != 100) {
			val = !val;
			val2 = !val2;

			// val = !val
			// j = j+1;
			// j != 200 ?
			// goto

		}

		// j = 0;
		// i = i+1;
		// i != ITERATIONS?
		//goto
	}

	// (5 * ITERATIONS) + 5*200*ITERATIONS


	total = (unsigned long)i * (unsigned long)j;
	unsigned int t = millis()-t0;
	//sprintf(buffer, "%lu operations in %u ms.", total, t);
	//Serial.println(buffer);
	sprintf(buffer, "[%lu - %d] %lu Kiterations/s", total, t, total/t);
	Serial.println(buffer);
}
