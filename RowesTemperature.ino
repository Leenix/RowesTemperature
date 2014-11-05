#include <Wire.h>
#include <Battery.h>
#include <DS3231.h>
#include <XBee.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "avr/wdt.h"
#include "avr/sleep.h"

//////////////////////////////////////////////////////////////////////////
// Configuration

#define UNIT_ID 1

// Pin assignments

#define SENSOR_POWER_PIN 3
#define TF_POWER_PIN 4
#define XBEE_POWER_PIN 5

#define ROOF_DT_PIN 6
#define LAWN_DT_PIN 7
#define AIR_DT_PIN 8


// Sensor configuration

#define TEMPERATURE_PRECISION 12	// Number of bits per temperature sample

OneWire roofWire(ROOF_DT_PIN);
DallasTemperature roofSensors(&roofWire);
int numRoofSensors;

OneWire lawnWire(LAWN_DT_PIN);
DallasTemperature lawnSensors(&lawnWire);
int numLawnSensors;

OneWire airWire(AIR_DT_PIN);
DallasTemperature airSensor(&airWire);
int numAirSensors;


// Transmit Buffer

#define PACKET_BUFFER_SIZE 100	// Maximum capacity of transmit buffer in bytes
byte packetBuffer[PACKET_BUFFER_SIZE];
byte bufferPutter;
byte bufferGetter;


// Xbee configuration

#define XBEE_WAKE_DELAY 2000	// Time given for XBee to initialize after powering up (ms)
XBee xbee = XBee();
XBeeAddress64 coordinatorAddress = XBeeAddress64(0, 0);
ZBTxRequest zbTx = ZBTxRequest(coordinatorAddress, packetBuffer, bufferPutter);
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

enum COMM_CODES{
	TEMPERATURE_READING = 'T',
	BATTERY_LEVEL = 'B',
	DATE_TIME_UPDATE = 'd',
	HOLD_AWAKE_NOTIFIER = 'h'
};


// Misc

DS3231 RTC;
Battery battery;
#define SAMPLE_PERIOD 5 // Time between samples (in minutes)
#define MAX_READINGS_PER_PACKET 5 // Max number of temperature sensor data readings per packet

//////////////////////////////////////////////////////////////////////////
// Main Functions
//////////////////////////////////////////////////////////////////////////

/**
* Initialiation - Run once
*/
void setup()
{
	enableWatchdog();
	
	// Start comms
	Serial.begin(57600);
	Wire.begin();
	RTC.begin();
	initialiseXBee();
	
	wdt_reset();
	
	Serial.println("eResearch - Rowes Bay Temperature Monitor");
	
	initialiseTemperatureSensors();
	
	disableWatchdog();
}


/**
* Main loop - Run forever
*/
void loop()
{
	// Start the watchdog. The code must reach the next watchdog checkpoint (reset) within 8 seconds.
	enableWatchdog();
	
	// Check for incoming packets
	//checkForIncomingPacket();
	
	Serial.print("\n\nAwake - ");
	printTimestamp();
	
	// Check sensors
	
	wdt_reset();
	Serial.println("\nRoof sensors: ");
	transmitTemperatureReadings(roofSensors, numRoofSensors);
	
	wdt_reset();
	Serial.println("\nLawn sensors: ");
	transmitTemperatureReadings(lawnSensors, numLawnSensors);
	
	wdt_reset();
	Serial.println("\nAir sensors: ");
	transmitTemperatureReadings(airSensor, numAirSensors);
	
	disableWatchdog();
	
	delay(1000);
	// Sleep until next sample period
	enterSleep();
}

//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
// Buffers

/**
* Send an integer to the buffer
* The value will be stored in its raw, byte form
*/
void toBuffer(unsigned int num){
	toBuffer(num);
}


/**
* Send an integer to the buffer
* The value will be stored in its raw, byte form
*/
void toBuffer(int num){
	byte high = highByte(num);
	byte low = lowByte(num);
	
	toBuffer(high);
	toBuffer(low);
}


/**
* Store the input byte into the buffer
* The putter position is incremented after a successful entry
*/
void toBuffer(byte b){
	if (bufferPutter < PACKET_BUFFER_SIZE){
		packetBuffer[bufferPutter] = b;
		bufferPutter++;
	}
}


/**
* Ready the buffer to accept new data
*/
void resetBuffer(){
	bufferPutter = 0;
	bufferGetter = 0;
}


/**
* Send the current timestamp to the buffer
* The data is entered into the buffer as 4 separate bytes
*/
void writeTimeToBuffer(){
	long timeStamp = RTC.now().get();
	
	byte b1 = (byte) timeStamp;
	byte b2 = (byte) (timeStamp >> 8 & 0xFF);
	byte b3 = (byte) (timeStamp >> 16 & 0xFF);
	byte b4 = (byte) (timeStamp >> 24 & 0xFF);
	
	toBuffer(b4);
	toBuffer(b3);
	toBuffer(b2);
	toBuffer(b1);
}

//////////////////////////////////////////////////////////////////////////
// Sensors

/**
* Start all temperature sensors and configure their resolution
*/
void initialiseTemperatureSensors(){
	pinMode(SENSOR_POWER_PIN, OUTPUT);
	powerUpSensors();
	
	roofSensors.begin();
	roofSensors.setResolution(TEMPERATURE_PRECISION);
	numRoofSensors = roofSensors.getDeviceCount();
	
	lawnSensors.begin();
	lawnSensors.setResolution(TEMPERATURE_PRECISION);
	numLawnSensors = lawnSensors.getDeviceCount();
	
	airSensor.begin();
	airSensor.setResolution(TEMPERATURE_PRECISION);
	numAirSensors = airSensor.getDeviceCount();
	
	Serial.print("Roof sensors detected: ");
	Serial.println(numRoofSensors);
	Serial.print("Lawn sensors detected: ");
	Serial.println(numLawnSensors);
	Serial.print("Air sensors detected: ");
	Serial.println(numAirSensors);
}


/**
* Write the addresses and temperature readings of a DS18B20
*/
void transmitTemperatureReadings(DallasTemperature &sensors, int numSensors){
	
	resetBuffer();
	toBuffer(byte(UNIT_ID));
	writeTimeToBuffer();
	
	sensors.requestTemperatures();
	
	// Write temperature addresses and readings to buffer in sequence
	for(int i = 0; i < numSensors; i++){
		DeviceAddress address;
		sensors.getAddress(address, i);
		int temperature = int(sensors.getTempCByIndex(i) * 100);
		
		// Enter data into buffer
		toBuffer(TEMPERATURE_READING);
		writeTempAddressToBuffer(address);
		toBuffer(',');
		toBuffer(temperature);
		toBuffer(',');
		
		Serial.print("Address: ");
		printAddress(address);
		Serial.print(", Temperature: ");
		Serial.println(float(temperature)/100);
		
		
		// Send the packet early if more than 5 entries have been added
		if((i+1) % MAX_READINGS_PER_PACKET == 0){
			toBuffer(readBatteryCapacity());
			transmitData();
			resetBuffer();
		}
	}
	
	toBuffer(readBatteryCapacity());
	transmitData();
}


/**
* Write the 64-bit DS18B20 address the buffer
*/
void writeTempAddressToBuffer(DeviceAddress address){
	for (int i = 0; i < 8; i++){
		toBuffer(address[i]);
	}
	
}


/**
* Print the address of a DS18B20 sensor in a readible format
*/
void printAddress(DeviceAddress deviceAddress){
	for (uint8_t i = 0; i < 8; i++){
		// zero pad the address if necessary
		if (deviceAddress[i] < 16) Serial.print("0");
		Serial.print(deviceAddress[i], HEX);
	}
}


/**
* Take readings from the battery charger
*/
int readBatteryCapacity(){
	battery.update();
	int batteryCapacity = battery.getPercentage();
	return batteryCapacity;
}


/**
* Provide power to the sensors
*/
void powerUpSensors(){
	digitalWrite(SENSOR_POWER_PIN, HIGH);
}


/**
* Cut power to the sensors
*/
void powerDownSensors(){
	digitalWrite(SENSOR_POWER_PIN, LOW);
}


//////////////////////////////////////////////////////////////////////////
// Sleep

/**
* Put the microcontroller into sleep mode until the sample period has elapsed
* Relies on the RTC using everyMinute interrupts
*/
void enterSleep()
{
	sleepNow();
	
	// Sleep Point //
	
	disableSleep();
	while ((RTC.now().minute() % SAMPLE_PERIOD) != 0){
		sleepNow();
		disableSleep();
	}
	
	wakeUp();
}


/**
* Put the microcontroller in a low power state
* Peripherals are powered down
*/
void sleepNow(){
	powerDownXbee();
	powerDownSensors();
	sleepController();
}


/**
* Wake the microcontroller and peripherals from sleep
*/
void wakeUp(){
	disableSleep();
	powerUpXbee();
	powerUpSensors();
}


/**
* Disable sleep from occurring
* Pin interrupts are detached and the safety is put back on
*/
void disableSleep(){
	detachInterrupt(INT0);
	sleep_disable();
}


/**
* Put the chip to sleep
* Only a reset, or a button interrupt can wake up the chip
*/
void sleepController()
{
	/*Initialize INT0 for accepting interrupts */
	PORTD |= 0x04;
	DDRD &=~ 0x04;
	
	// Lowest level sleep - Highest power savings
	// The microcontroller can only be woken by reset or external interrupt
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	
	// Enable RTC interrupts
	RTC.clearINTStatus();
	attachInterrupt(INT0, clockInterrupt, LOW);
	RTC.enableInterrupts(EveryMinute);
	
	// Take the safety off
	sleep_enable();
	
	// Put the controller to sleep
	sleep_mode();
}


/**
* Dummy function for RTC interrupt
*/
void clockInterrupt(){
	// Dummy function for RTC interrupt
}


void printTimestamp(){
	DateTime timestamp = RTC.now();
	
	Serial.print(timestamp.year());
	Serial.print("-");
	if(timestamp.month() < 10){Serial.print(0);}
	Serial.print(timestamp.month());
	Serial.print("-");
	if(timestamp.date() < 10){Serial.print(0);}
	Serial.print(timestamp.date());
	Serial.print(" ");
	if(timestamp.hour() < 10){Serial.print(0);}
	Serial.print(timestamp.hour());
	Serial.print(":");
	if(timestamp.minute() < 10){Serial.print(0);}
	Serial.print(timestamp.minute());
	Serial.print(":");
	if(timestamp.second() < 10){Serial.print(0);}
	Serial.println(timestamp.second());
}

//////////////////////////////////////////////////////////////////////////
// Watchdog

/**
* Enable the watchdog timer - Default timeout of 8 seconds
*/
void enableWatchdog(){
	cli();
	
	wdt_reset();
	
	wdt_enable(WDTO_8S);

	sei();
}


/**
* Disable the watchdog timer
*/
void disableWatchdog(){
	cli();
	
	wdt_reset();
	
	wdt_disable();
	
	sei();
}


//////////////////////////////////////////////////////////////////////////
// XBee

/**
* Set up the XBee radio
*/
void initialiseXBee(){
	pinMode(XBEE_POWER_PIN, OUTPUT);
	powerUpXbee();
	
	xbee.setSerial(Serial);
}


/**
* Check if there are any incoming packets from the XBee
* Process any receive packets
*/
void checkForIncomingPacket(){
	xbee.readPacket(100);
	
	if (xbee.getResponse().isAvailable()){
		//TODO get packet
	}
}


/**
* Process incoming packets
* First character contains the
*/
void handleIncomingPacket(){
	char packetType;
	//TODO get packet type from frame
	
	switch(packetType){
		case HOLD_AWAKE_NOTIFIER:
		//TODO Hold awake
		break;
		
		case DATE_TIME_UPDATE:
		//TODO update datetime
		break;
	}
}


/**
* Send the recorded packet over the XBee
* Uses API mode transmission
*/
void transmitData(){
	zbTx = ZBTxRequest(coordinatorAddress, packetBuffer, bufferPutter);
	xbee.send(zbTx);
	
	Serial.println("\nPacket sent");
}


/**
* Stop power to the XBee radio
*/
void powerDownXbee(){
	digitalWrite(XBEE_POWER_PIN, LOW);
}


/**
* Supply power to the XBee radio
*/
void powerUpXbee(){
	digitalWrite(XBEE_POWER_PIN, HIGH);
	delay(XBEE_WAKE_DELAY);
}
