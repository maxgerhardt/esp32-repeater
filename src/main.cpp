#include <Arduino.h>
#include <SoftwareSerial.h>

/* "Serial" / UART0 is used for Arduino IDE debug messages (GPIO1, GPIO3 as TX,RX)
 * "Serial1" / UART1 is on GPIO9 and GPIO10 which doesn't seem to be exposed on most boards
 * "Serial2" / UART2 on GPIO16 and GPIO17 which is exposed
 * Use last hardware serial to read and write from paradox device.
 * Use software serials to read and write from other 2 devices
 * */

/**
 * GPIO 9 (RX) GPIO 10 (TX)
 */
#define PARADOX_UART Serial2

/**
 * IO32, IO33
 */
#define ESP8266_UART_RX 32
#define ESP8266_UART_TX 33

/**
 * IO25, 26
 */
#define G16_UART_RX 25
#define G16_UART_TX 26

SoftwareSerial espSerial(ESP8266_UART_RX, ESP8266_UART_TX);
SoftwareSerial g16Serial(G16_UART_RX, G16_UART_TX);

/**
 * Statics
 */
int numRxParadox = 0;
int numTxParadox = 0;
int numTxEsp = 0;
int numRxEsp = 0;
int numTXG16 = 0;
int numRXG16 = 0;
const int printStatsIntervalMillis = 1000;
int prevMillis = 0;

void setup() {
	//debug serial hardwired to
	Serial.begin(115200);
	PARADOX_UART.begin(9600, SERIAL_8N1);
	espSerial.begin(9600, SoftwareSerialConfig::SWSERIAL_8N1);
	g16Serial.begin(9600, SoftwareSerialConfig::SWSERIAL_8N1);
}

void loop() {
	//check if something received from paradox, forward to
	//the slave devices
	if(PARADOX_UART.available() > 0) {
		uint8_t rcvd = (uint8_t) PARADOX_UART.read();
		espSerial.write(rcvd);
		g16Serial.write(rcvd);

		numRxParadox++; numTxEsp++; numTXG16++;
	}
	//check back-channels, forward to paradox
	if(espSerial.available() > 0) {
		uint8_t rcvd = (uint8_t) espSerial.read();
		PARADOX_UART.write(rcvd);

		numRxEsp++; numTxParadox++;
	}
	if(g16Serial.available() > 0) {
		uint8_t rcvd = (uint8_t) g16Serial.read();
		PARADOX_UART.write(rcvd);

		numRXG16++; numTxParadox++;
	}

	//display stats
	unsigned long currentMillis = millis();
	if (currentMillis - prevMillis >= printStatsIntervalMillis) {
		prevMillis = currentMillis;
		Serial.println("==== INTERFACE STATISTICS =====");
		Serial.println("Paradox: RX " + String(numRxParadox) + " Bytes TX " + String(numTxParadox) + " Bytes");
		Serial.println("ESP8266: RX " + String(numRxEsp) + " Bytes TX " + String(numTxEsp) + " Bytes");
		Serial.println("G16    : RX " + String(numRXG16) + " Bytes TX " + String(numTXG16) + " Bytes");
	}
}
