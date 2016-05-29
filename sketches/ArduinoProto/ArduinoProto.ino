// This demo does web requests via DHCP and DNS lookup.
// 2011-07-05 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <EtherCard.h>
//#include <dht.h>

//dht DHT;

//#define DHT22_PIN 5

#define REQUEST_RATE 5000 // milliseconds

static byte myip[] = { 192, 168, 2, 200 };
// gateway ip address
static byte gwip[] = { 192, 168, 2, 1 };

// ethernet interface mac address
static byte mymac[] = { 0x74, 0x69, 0x69, 0x2D, 0x30, 0x66 };
// remote website name
const char website[] PROGMEM = "google.es";

byte Ethernet::buffer[128];
static long timer;

// called when the client request is complete
static void my_result_cb (byte status, word off, word len) {
  Serial.print("<<< reply ");
  Serial.print(millis() - timer);
  Serial.println(" ms");
  Serial.println((const char*) Ethernet::buffer + off);
}

void setup () {
  Serial.begin(115200);
  Serial.println("\n[getDHCPandDNS]");

  if (ether.begin(sizeof Ethernet::buffer, mymac) == 0)
    Serial.println( "Failed to access Ethernet controller");

  ether.staticSetup(myip, gwip, gwip);
  //if (!ether.dhcpSetup())
  //  Serial.println("DHCP failed");

  ether.printIp("My IP: ", ether.myip);
  // ether.printIp("Netmask: ", ether.mymask);
  ether.printIp("GW IP: ", ether.gwip);
  ether.printIp("DNS IP: ", ether.dnsip);

  if (!ether.dnsLookup(website))
    Serial.println("DNS failed");

  ether.printIp("Server: ", ether.hisip);

  timer = - REQUEST_RATE; // start timing out right away
}

uint32_t m = 0;

void loop () {
/*
	if((millis() - m) > 1000) {
		char buffer[32];
		m = millis();
		Serial.println("Reading temp&hum...");
		DHT.read22(DHT22_PIN);
		sprintf(buffer, "%02f ºC, %02f %", DHT.temperature, DHT.humidity);
		Serial.println(buffer);

	}
*/
	

	ether.packetLoop(ether.packetReceive());

	if (millis() > timer + REQUEST_RATE) {
	timer = millis();
	Serial.println("\n>>> REQ");
	ether.browseUrl(PSTR("/"), "", website, my_result_cb);
	}
}