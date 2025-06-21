/*
Copyright 2024, 2025 Jonathan Seyfert

This file is part of Indoor-Outdoor_Temp_Sensor.

Indoor-Outdoor_Temp_Sensor is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License
as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

Indoor-Outdoor_Temp_Sensor is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Indoor-Outdoor_Temp_Sensor. If not, see <https://www.gnu.org/licenses/>. 
*/

#include <SPI.h> // For RFM69
#include <RH_RF69.h> // For RFM69
#include <RHReliableDatagram.h> // For RFM69
#include <Adafruit_SHT4x.h> // for SHT45 temp/humidity sensor
#include <Adafruit_SleepyDog.h> // for watchdog sleep for power savings
#include <json.hpp> // for handling jsons (nholmann::json)
// Must place json.hpp in your Arduino IDE libraries folder in a folder called "json". 
// When including in sketch, it must come after including RH_RF69.h or you must include Arduino.h AFTER this library or compiling will fail due to how Arduino defines abs() with a macro

const uint16_t rfm69Freq = 915; // Frequency of RFM69
const uint8_t rfm69Cs = 8; // RFM69 SPI chip select pin
const uint8_t rfm69Int = 3; // RFM69 interrupt pin
const uint8_t rfm69Rst = 4; // RFM69 reset pin
const uint8_t led = 13; // Built-in LED on M0 Feather
const uint8_t vBat = A7; // Internal battery voltage divider measurement pin
const uint8_t shtPwr = 5; // Pin to supply power to SHT45 to turn it on and off for power savings
extern "C" char *sbrk(int i);	// for FreeMem()

enum Address // addresses for all units in the network
{
	base = 0,
	outdoor = 1,
	basement = 2,
};
Address thisUnit { outdoor }; // Set this to the enum in the list above for which unit this is

Adafruit_SHT4x sht4 = Adafruit_SHT4x(); // for SHT45
RH_RF69 driver(rfm69Cs, rfm69Int); // Singleton instance of the radio driver
RHReliableDatagram manager(driver, thisUnit); // Class to manage message delivery and receipt, using the driver declared above

// #define SERIAL_DEBUG // Uncomment if serial debugging is desired. Watchdog sleep will be disabled if defined and replaced with delay()

void setup() {
	#ifdef SERIAL_DEBUG
	Serial.begin(115200);
	#endif

pinMode(rfm69Rst, OUTPUT);
	digitalWrite(rfm69Rst, LOW);

	// manual reset RFM69
	digitalWrite(rfm69Rst, HIGH);
	delay(10);
	digitalWrite(rfm69Rst, LOW);
	delay(10);

	pinMode(led, OUTPUT);

	pinMode(shtPwr, OUTPUT); // Initialize pin for SHT45 power control
	digitalWrite(shtPwr, HIGH); //turn on SHT45 for initialization
	delay(1); // chip needs 1 ms delay on power up before it's ready
	sht4.begin(); // initialize SHT45 sensor
	sht4.setPrecision(SHT4X_HIGH_PRECISION); // can use MED or LOW, HIGH takes longer
	sht4.setHeater(SHT4X_NO_HEATER); // can use 6 different heater options, see example

	manager.init(); // Initialize RFM69
	driver.setFrequency(rfm69Freq); // Set RFM69 frequency
	driver.setTxPower(20, true);	// Power levels can be set from -2 to 20, 2nd arg must be true for RFM69. 1% duty cycle at 20, VSWR 3:1 max
	uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}; // encryption key, must be set as same as main unit
	driver.setEncryptionKey(key);
}

void loop()
{
	sensors_event_t humidity, temp; // for SHT45
	digitalWrite(shtPwr, HIGH); // turn on SHT45
	delay(1); // sensor needs 1 ms after power up before it's ready
	sht4.begin(); // initialize SHT45 sensor
	sht4.setPrecision(SHT4X_HIGH_PRECISION); // can use MED or LOW, HIGH takes longer
	sht4.setHeater(SHT4X_NO_HEATER); // can use 6 different heater options, see example
	if(sht4.getEvent(&humidity, &temp)) // check if our request for temp and humidity worked
	{
		nlohmann::json message; // empty json struction to hold message we want to send
		message["dry_bulb"] = round(temp.temperature*10)/10.0; // add dry bulb temp to json, rounding to 1 decimal place
		message["humidity"] = round(humidity.relative_humidity*10)/10.0; // add RH to json, rounding to 1 decimal place
		message["wet_bulb"] = round(wetBulbCalc(temp.temperature, humidity.relative_humidity)*10)/10.0; // add web bulb temp to json, round to 1 decimal place
		message["battery_voltage"] = round(analogRead(vBat)*0.006445*100)/100; // ADC*2*3.3/1024 *2 - 50% voltage divider. *3.3 - V_ref. /1024 - 8-bit ADC. Round 2 decimal places
		String jsonMsg = String(message.dump().c_str()); // convert the json object into a String

		uint8_t numPacs = jsonMsg.length()/57 + 1; // each packet sent can be 60 characters long, but need to leave one char for EOM, so we get 59 sendable chars per packet, - 2 for X of Y pac #
		char packets[numPacs][60] = {'\0'}; // array of char arrays to hold packets for sending
		for (uint8_t i = 0; i < numPacs; i++) // sub-device json String into 59 char log strings, then convert sub-strings to char arrays and feed into array of char packets
		{
			String s = String(String(char(i + 1)) + String(char(numPacs))); // packet header: X packet of Y # packets. Typecast first and second chars of packet to int to decode on Rx.
			s += jsonMsg.substring(i*57, i*57 + 57); // concat 57 chars of message after X of Y header, leaving 1 char for Null termination of char string
			s.toCharArray(packets[i], 60); // Convert String to char string for sending via packet radio
		}

		#ifdef SERIAL_DEBUG
		Serial.print("json contents: ");
		Serial.println(jsonMsg);
		for (uint8_t i = 0; i < numPacs; i++)
		{
			Serial.println(packets[i]);
		}
		Serial.print("free memory: ");
		Serial.println(FreeMem());
		#endif

		bool pac[numPacs];
		bool allSent = false;
		for (uint8_t i; i < numPacs; i++) // send all packets
		{
			if (!manager.sendtoWait((uint8_t *)packets[i], sizeof(packets[i]), base))
			{
				allSent = true;
			}
		}
		if (allSent == false)
		{
			digitalWrite(led, LOW); // if the LED was previously on to indicate a problem, turn it off now
		}
		else
		{
			digitalWrite(led, HIGH); // turn on LED to indicate failure to recieve ack
		}
	}
	else
	{
		digitalWrite(shtPwr, HIGH); // turn on LED to indicate a problem

		#ifdef SERIAL_DEBUG
		Serial.print("failed to get event from SHT45");
		#endif
	}
	digitalWrite(shtPwr, LOW); // turn off SHT45 for power saving while sleeping
	
	#ifdef SERIAL_DEBUG // If we're debugging, delay. Watchdog sleep makes it difficult to upload code changes
	delay(16000);
	#else // watchdog sleep when not debugging, for power savings
	Watchdog.sleep(16000); // sleep for 16 seconds (16 is maximum, but if you do between 8000 and 16000 will drop to 8)
	#endif
}

float wetBulbCalc(float DB, float RH)
{
	float WBC = DB*(atan(0.151977*pow(RH + 8.313659,0.5))) + atan(DB + RH) - atan(RH - 1.676331) + 0.00391838*pow(RH, 1.5)*atan(0.023101*RH) - 4.686035;
	return WBC;
}

int FreeMem () // http://forum.arduino.cc/index.php?topic=365830.msg2542879#msg2542879
{
	char stack_dummy = 0;
	return &stack_dummy - sbrk(0);
}
