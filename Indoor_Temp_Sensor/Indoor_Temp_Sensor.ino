/*
Copyright 2024, 2025 Jonathan Seyfert

This file is part of Indoor-Outdoor_Temp_Sensor.

Indoor-Outdoor_Temp_Sensor is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License
as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

Indoor-Outdoor_Temp_Sensor is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Indoor-Outdoor_Temp_Sensor. If not, see <https://www.gnu.org/licenses/>. 
*/

#include <Adafruit_GFX.h> // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include "Adafruit_SHT4x.h" // for SHT45 temp/humidity sensor
#include <SPI.h> // For RFM69 & LCD & SD card
#include <RH_RF69.h> // For RFM69
#include <RHReliableDatagram.h> // FOr RFM69
#include <SD.h> // For SD card logging
#include "RTClib.h" // For RTC
#include "Button.h" //for buttons
#include <json.hpp>
//Must place json.hpp this in your Arduino IDE libraries folder in a folder called "json". 
//When including in sketch, it must come after including RH_RF69.h or you must include Arduino.h AFTER this library or compiling will fail due to how Arduino defines abs() with a macro

#define MENU_SELECT ST77XX_WHITE, ST77XX_BLACK
#define MENU_UNSELECT ST77XX_BLACK, ST77XX_WHITE
#define SERIAL_DEBUG // Enables various serial debugging messages if defined

// The below are the pin number variables. Update based on microcontroller and usage
const uint8_t vBat = A7;		// 9/A7 - Internal battery voltage divider measurement pin
const uint8_t rfm69Cs = 8;		// RFM69 pins on M0 Feather
const uint8_t rfm69Int = 3;		// RFM69 pins on M0 Feather
const uint8_t rfm69Rst = 4;		// RFM69 pins on M0 Feather
const uint8_t downPin = 5;		// Down button pin
const uint8_t upPin = 6;		// Up button pin
const uint8_t selectPin = 14;	// (A0) Select button pin 
const uint8_t tftBacklight = 15;// (A1) pin for backlight PWM
const uint8_t tftDc = 10;		// DC for LCD display
const uint8_t tftCs = 11;		// Chip select for LCD display
const uint8_t tftRst = 12;		// Reset for LCD display
const uint8_t led = 13;			// Built-in LED pin, also GPIO if sharing the LED is cool
const uint8_t sdCs = 16;		// CS for RTC datalogging wing SD card

/*
M0 USED PINS NOT DEFINED ABOVE:
	For SPI interface (LCD/SD card/RFM69):
		24 (SCK)
		23 (MOSI)
		22 (MISO)
	For I2C interface (SHT45)
		21 (SCL)
		20 (SDA)
	For both SPI and I2C, the above pins are the hardware interface pins, used unless you specifically call other pins.

M0 UNUSED PINS:
	17/A3
	18/A4
	19/A5
	0
	1

RFM69 UNUSED PINS:
	DIO1
	DIO2 // header not soldered, but through-hole available
	DIO3 // header not soldered, but through-hole available
	DIO5 // header not soldered, but through-hole available
	These are GPIO pins on the RFM69 module itself, available to use with serial commands to RFM69
*/

// The below variables are config variables. Adjust to suit needs
// For built-in LED watchdog pulsing:
int8_t fadeAmount = 10; 				// How much to adjust brightness during each fade step. Not const as it flips from positive to negative
const uint8_t watchdogFadeTime = 100;	// how often to adjust brightness by the fadeAmount each fade step

// For LCD backlight:
uint8_t tftBrightLevel = 4; // The brightness of the display, an element in tftBrightness[]. Initialized value is valued used on boot and can be adjusted later
const uint8_t tftBrightness[] = {1, 10, 40, 80, 160, 255}; // brightness setpoints the TFT display backlight. Can add or remove elements 

// For update rates:
const uint16_t updateTime = 5000;	// How often to update display
const uint16_t altDisplayTime = 10000; // time to display alternate screens (min/max, alternate sensor)
const uint16_t minMaxDisplayTime = 15000; // How long to display the Min/Max screen
const uint32_t logTime = 60000; // how often to log data

// For RFM69 radio network:
const uint16_t rfm69Freq = 915;	// RFM69 frequency (MHz). Verify legality of frequency in your area. All units in network must use same frequency
uint8_t encryptionKey[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // encryption key for RMF69 radio. Must match that on remote sensor!
							0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}; // should change the default encryption key...
enum Address // addresses for all units in network. Update as needed
{
	base = 0,
	outdoor = 1,
	basement = 2,
};
Address thisUnit { base }; // Address of this unit. Should likely NOT be changed, as this base unit has considerably different code than the remote sensors, which are fairly similar

// The below variables are not config variables
extern "C" char *sbrk(int i);			// for FreeMem()
int16_t ledBrightness = 0;				// brightness of the built-in LED
uint32_t timer = 0;				// Used to check if it's time to update display
uint32_t logTimer = 0;				// used to check if it's time to log data
uint32_t watchdogBlinkTimer = 0;	// used to check if it's time to adjust brightness of watchdog LED
nlohmann::json j;						// one json to hold all variables from all units
String rxData;							// String to assemble received packets into

DateTime bootTime; // To display time at boot, time for min/max
RTC_DS3231 rtc; // for RTC
Button buttonUp(upPin); // for an up button. Defaults to 100 ms debounce, use (pin, debounce_ms) to change
Button buttonDown(downPin); // for a down button
Button buttonSelect(selectPin); // for a down button
Adafruit_SHT4x sht4 = Adafruit_SHT4x(); // for SHT45
Adafruit_ST7789 display = Adafruit_ST7789(tftCs, tftDc, tftRst); // for LCD
RH_RF69 driver(rfm69Cs, rfm69Int); // Singleton instance of the radio driver
RHReliableDatagram manager(driver, thisUnit); // Class to manage message delivery and receipt, using the driver declared above

void setup()
{
	#ifdef SERIAL_DEBUG
	Serial.begin(115200);	// for testing
	#endif

	// Initialize the buttons using the button library
	buttonUp.begin();
	buttonDown.begin();
	buttonSelect.begin();

	pinMode(vBat, INPUT);
	// When not using RFM, uncomment the below two lines to keep pin 8 RFM69 CS high, otherwise RMF69 will conflict with other SPI devices
	// pinMode(8, OUTPUT);	
	// digitalWrite(8, HIGH);

	pinMode(led, OUTPUT); // Configure LED pin as output
	analogWrite(led, ledBrightness); // Give LED an initial value
	pinMode(tftBacklight, OUTPUT);
	analogWrite(tftBacklight, tftBrightness[tftBrightLevel]); // set lcd to full selected brightness to start

	// reset RFM69
	pinMode(rfm69Rst, OUTPUT);	
	digitalWrite(rfm69Rst, LOW);	// is this needed?
	digitalWrite(rfm69Rst, HIGH);
	delay(10);
	digitalWrite(rfm69Rst, LOW);
	delay(10);

	manager.init(); // Initialize RFM69 radio
	driver.setFrequency(rfm69Freq); // Set RFM69 radio frequency
	driver.setTxPower(20, true); // Power levels can be set from -2 to 20, 2nd arg must be true for RFM69. 1% duty cycle at 20, VSWR 3:1 max
	driver.setEncryptionKey(encryptionKey);	// Set RFM69 encryption key
	SPI.usingInterrupt(digitalPinToInterrupt(rfm69Int)); // The RadioHead library doesn't register it does SPI within an interrupt, so this is required to avoid conflicts
	
	display.init(170, 320); // Initialize the LCD ST7789 (170x320 pixels)
	display.setRotation(3); // 0 & 2 are portrait, 1 & 3 are landscape
	display.fillScreen(ST77XX_WHITE); // fill screen with white because we draw with black
	delay(100);	// Give the screen a sec to initialize

	if(!sht4.begin()) // initialize SHT45 sensor
	{
		#ifdef SERIAL_DEBUG
		Serial.println(F("Unable to initialize SHT45!"));
		#endif
		displayError(F("Unable to initialize SHT45!"));
	}
	sht4.setPrecision(SHT4X_HIGH_PRECISION); // can use MED or LOW, HIGH takes longer
	sht4.setHeater(SHT4X_NO_HEATER); // can use 6 different heater options, see example
 
	if (!SD.begin(sdCs)) // initialize the SD card
	{
		#ifdef SERIAL_DEBUG
		Serial.println(F("Card init. failed!"));
		#endif
		displayError(F("Unable to initialize SD card!"));
	}

	if(!rtc.begin()) // initialize RTC
	{
		#ifdef SERIAL_DEBUG
		Serial.println(F("RTC init. failed!")); // print error over USB Serial port
		#endif
		displayError(F("Unable to initialize RTC!")); // display error on screen
	}
	//rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // adjusts date and time to that of sketch compiling. Comment out after first upload
	bootTime = rtc.now(); // Record the DateTime of boot

	// The below updates the display once on power-up
	sensors_event_t humidity, temp; // for SHT45
	sht4.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data
	float dryBulb = temp.temperature; // Get indoor temp
	// j["units"][base]["min_db"] = round(dryBulb*100)/100.0;
	// j["units"][base]["max_db"] = round(dryBulb*100)/100.0;
	float wetBulb = wetBulbCalc(dryBulb, humidity.relative_humidity);
	updateDisplay(dryBulb*1.8 + 32, humidity.relative_humidity, wetBulb*1.8 + 32);
}

void loop()
{
	if (manager.available()) // we've received a packet
	{
		uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
		uint8_t len = sizeof(buf);
		uint8_t from;
		if (manager.recvfromAck(buf, &len, &from))
		{
			if (!len) return; // checks to see if message length is 0. Unclear why that's needed, since rf69.available should not return true if there is no message?
			//buf[len] = 0; // what does this do?
			String rxPacket = ((char*)buf); // transfer message into String variable
			handleRxPacket(buf, from);
			updateMinMaxDB(from); // to update min/max values for this sensor
		}
	}
	
	if(millis() - timer >= updateTime) // run every updateTime ms interval
	{
		sensors_event_t humidity, temp; // for SHT45
		sht4.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data
		nlohmann::json indoor;
		indoor["dry_bulb"] = round(temp.temperature*10)/10.0; // add dry bulb temp to json, rounding to 1 decimal place
		indoor["humidity"] = round(humidity.relative_humidity*10)/10.0; // add RH to json, rounding to 1 decimal place
		indoor["wet_bulb"] = round(wetBulbCalc(temp.temperature, humidity.relative_humidity)*10)/10.0; // add web bulb temp to json, round to 1 decimal place
		indoor["battery_voltage"] = round(analogRead(vBat)*0.006445*100)/100; // ADC*2*3.3/1024 *2 - 50% voltage divider. *3.3 - V_ref. /1024 - 8-bit ADC. Round 2 decimal places
		updateJson(indoor, 0);
		updateMinMaxDB(base);
		updateDisplay((j["units"][0]["dry_bulb"].template get<float>())*1.8 + 32, j["units"][0]["humidity"].template get<float>(), (j["units"][0]["wet_bulb"].template get<float>())*1.8 + 32); // update display with current readings
		timer = millis(); // reset timer so this function is called at appropriate timing
	}

	if(millis() - logTimer >= logTime) // log data every "logTime" ms interval
	{
		logData();
		logTimer = millis(); // reset timer so this function is called at appropriate timing
	}

	if(buttonUp.pressed())
	{
		displayMinMax();
		delay(altDisplayTime);
	}
	if(buttonSelect.pressed())
	{
		Serial.println(F("opening config menu from main loop"));
		configurationMenu();
	}
	if(buttonDown.pressed())
	{
		// currently no action, used to display key data. Should display copy of main display but with two additional units
	}

	if(millis() - watchdogBlinkTimer >= watchdogFadeTime)
	{
		ledBrightness += fadeAmount;
		if (ledBrightness <= 0)
		{
			ledBrightness = 0;
			fadeAmount = -fadeAmount;
		}
		if (ledBrightness >= 255)
		{
			ledBrightness = 255;
			fadeAmount = -fadeAmount;
		}
		analogWrite(led, ledBrightness);
		watchdogBlinkTimer = millis();
	}
}

void handleRxPacket(uint8_t buf[60], uint8_t from)
{
	uint8_t num = uint8_t(buf[0]); // first char is packet number
	uint8_t numPac = uint8_t(buf[1]); // second char is packet total
	uint8_t buf2[58] = {'\0'};
	for (uint8_t i = 0; i < 58; i++)
	{
		buf2[i] = buf[i + 2];
	}
	rxData += (char*)buf2; // copy char to String that concats received data
	if (num == numPac) // if last packet then transfer to json and wipe rxData String
	{
		#ifdef SERIAL_DEBUG
		Serial.print("Json contents before rx: ");
		Serial.println(j.dump().c_str());
		Serial.print("Reconstructed rx string: ");
		Serial.println(rxData);
		#endif

		if(nlohmann::json::accept(rxData)) // accept() returns true if valid json String, to avoid parse() throwing error if parsing invalid Json
		{
			nlohmann::json jRx = nlohmann::json::parse(rxData); // temp json object to hold received data
			jRx["RSSI"] = driver.lastRssi();
			rxData = ""; // wipe string
			return updateJson(jRx, from);
		}
		else // need to wipe string if the data is bad or it will just keep appending new data on the string
		{
			rxData = ""; // wipe string
			#ifdef SERIAL_DEBUG
			Serial.println("Bad JSON parsing from rxString, wiping string");
			#endif
		}
	}
}

void updateJson(nlohmann::json json, uint8_t unit)
{
		nlohmann::json jt; // temp json object to hold time of rx
		DateTime now = rtc.now();
		jt["year"] = now.year();
		jt["month"] = now.month();
		jt["day"] = now.day();
		jt["hour"] = now.hour();
		jt["minute"] = now.minute();
		jt["second"] = now.second();
		j["units"][unit].update(jt); // update time in global json object for this unit
		j["units"][unit].update(json); // update global json contents with newly recieved data
		

		#ifdef SERIAL_DEBUG
		Serial.print("Json contents after update: ");
		Serial.println(j.dump().c_str());
		#endif
}

void updateDisplay(float DB, float RH, float WB)
{
	if (j["units"][0] == nullptr || j["units"][1] == nullptr)
		return;
	float outsideDB = j["units"][1]["dry_bulb"].template get<float>()*1.8 + 32;
	float outsideWB = j["units"][1]["wet_bulb"].template get<float>()*1.8 + 32;
	float outsideRH = j["units"][1]["humidity"].template get<float>();
	float outsideWBGT = 0.7*outsideWB + 0.3*outsideDB;
	float outsideBat = j["units"][1]["battery_voltage"].template get<float>();
	int8_t outsideRSSI = j["units"][1]["RSSI"].template get<int8_t>();
	display.fillScreen(ST77XX_WHITE);
	display.setTextSize(2);
	display.setTextColor(ST77XX_BLACK);
	display.setCursor(5, 5);
	display.print(F("         Indoor  Outdoor"));

	display.setCursor(5, 30);
	display.print(F("Temp [F]  "));
	display.print(DB, 1);
	display.print(F("    "));
	display.print(outsideDB, 1);

	display.setCursor(5, 55);
	display.print(F("RH   [%]  "));
	display.print(RH, 1);
	display.print(F("    "));
	display.print(outsideRH, 1);

	display.setCursor(5, 80);
	display.setTextColor(ST77XX_BLACK);
	display.print(F("WBGT [F]  "));
	display.print(0.7*WB + 0.3*DB, 1);
	display.print(F("    "));
	display.print(outsideWBGT, 1);

	display.setCursor(5, 105);
	display.setTextColor(ST77XX_BLACK);
	display.print(F("Bat  [V]  "));
	display.print(j["units"][base]["battery_voltage"].template get<float>(), 2);
	display.print(F("    "));
	display.print(outsideBat, 2);

	display.setCursor(5, 130);
	display.setTextColor(ST77XX_BLACK);
	display.print(F("RSSI      "));
	display.print(F(" N/A"));
	display.print(F("     "));
	display.print(outsideRSSI);

	#ifdef SERIAL_DEBUG
	display.setCursor(5, 155);
	display.setTextColor(ST77XX_BLACK);
	display.print(F("MEM: "));
	display.print(FreeMem());
	#endif

	display.drawFastVLine(195, 0, 153, ST77XX_BLACK);
	display.drawFastVLine(105, 0, 153, ST77XX_BLACK);
	display.drawFastHLine(0, 25, 296, ST77XX_BLACK);

	// DateTime now = rtc.now(); // Get the current time to update the display with
	display.setTextSize(1);
	display.setTextColor(ST77XX_BLACK);
	display.setCursor(5, 4);
	display.print(F("Last update:"));
	display.setCursor(5, 14);
	display.print("I-"); // indoor display update time
	if (j["units"][0]["hour"].template get<uint8_t>() < 10) {
		display.print("0");
	}
	display.print(j["units"][0]["hour"].template get<uint8_t>(), DEC);
	display.print(":");
	if (j["units"][0]["minute"].template get<uint8_t>() < 10) {
		display.print("0");
	}
	display.print(j["units"][0]["minute"].template get<uint8_t>(), DEC);
	display.print("  O-"); // outdoor display update time
	if (j["units"][1]["hour"].template get<uint8_t>() < 10) {
		display.print("0");
	}
	display.print(j["units"][1]["hour"].template get<uint8_t>(), DEC);
	display.print(":");
	if (j["units"][1]["minute"].template get<uint8_t>() < 10) {
		display.print("0");
	}
	display.print(j["units"][1]["minute"].template get<uint8_t>(), DEC);

}

float wetBulbCalc(float DB, float RH)
{
	float WBC = DB*(atan(0.151977*pow(RH + 8.313659,0.5))) + atan(DB + RH) - atan(RH - 1.676331) + 0.00391838*pow(RH, 1.5)*atan(0.023101*RH) - 4.686035;
	return WBC;
}

int FreeMem ()
{	//http://forum.arduino.cc/index.php?topic=365830.msg2542879#msg2542879
	char stack_dummy = 0;
	return &stack_dummy - sbrk(0);
}

void displayMinMax()
{
	display.fillScreen(ST77XX_WHITE);
	display.setCursor(5, 5);
	display.setTextSize(2);
	display.setTextColor(ST77XX_BLACK);
	display.print(F("MIN/MAX"));
	display.setCursor(5, 30);
	display.setTextSize(1);
	display.print(F("Since "));
	display.print(bootTime.year(), DEC);
	display.print(F("-"));
	display.print(bootTime.month(), DEC);
	display.print(F("-"));
	display.print(bootTime.day(), DEC);
	display.print(F(" "));
	display.print(bootTime.hour(), DEC);
	display.print(F(":"));
	display.print(bootTime.minute(), DEC);

	display.setCursor(5, 45);
	display.setTextSize(2);
	display.print(F("Indoor:  "));
	display.print(j["units"][base]["min_db"].template get<float>()*1.8 + 32);
	display.print(F("/"));
	display.print(j["units"][base]["max_db"].template get<float>()*1.8 + 32);

	display.setCursor(5, 70);
	display.print(F("Outdoor: "));
	display.print(j["units"][outdoor]["min_db"].template get<float>()*1.8 + 32);
	display.print(F("/"));
	display.print(j["units"][outdoor]["max_db"].template get<float>()*1.8 + 32);

}

void logData() // just log the entire json to file
{
	String logMessage = String(j.dump().c_str()); // convert JSON object into a String
	File logFile = SD.open("datalog", FILE_WRITE); // Open file for logging crash as writable file
	if(logFile)
	{
		logFile.println(logMessage); // print to file
		logFile.close(); // close file to ensure it was written
		#ifdef SERIAL_DEBUG
		Serial.println(F("Logged to datalog.txt!")); // for debug
		#endif
	}
	else
	{
		displayError(F("Unable to open datalog.txt!")); // display error
		#ifdef SERIAL_DEBUG
		Serial.println(F("Unable to open datalog.txt!")); // for debug
		#endif
	}
	#ifdef SERIAL_DEBUG
	Serial.println(logMessage); // for debug
	#endif

}

void displayError(String error)
{
	display.fillScreen(ST77XX_WHITE);
	display.setCursor(5, 5);
	display.setTextSize(2);
	display.setTextColor(ST77XX_BLACK);
	display.print(F("ERROR! "));
	display.print(error);
	display.setCursor(5, 55);
	display.setTextSize(1);
	display.print(F("Press button C to continue and ignore error."));
	display.setCursor(5, 70);
	display.print(F("The item that errored will likely not work until fixed."));
	delay(500); // for some reason, without a delay here, calling display() would hang the micro/crash it

	while(!buttonUp.pressed()) // stop on this screen until button C is pressed

	display.setCursor(5, 100);
	display.print(F("Button C pressed, continuing to proceed, ignoring error..."));
	delay(5000); // pause for reading time before proceeding
}

void configurationMenu()
{
	Serial.println("Top of config menu");
	display.fillScreen(ST77XX_WHITE);
	display.setTextSize(2);
	display.setTextColor(ST77XX_BLACK);
	display.setCursor(5, 5);
	display.print(F("Configuration Menu"));

	int menuCursorPos = 0; // what menu list option should be highlighted with cursor?
	int menuPos[] = {35, 55, 75}; // y-position values for menu entries
	String menuString[] = {	"Backlight brightness", // list of menu entries
							"Other setting			 ",
							"Other setting 2		 "};

	while(1)
	{
		bool exitLoop = false;
		int menuLength = sizeof(menuPos)/sizeof(menuPos[0]);
		if(menuCursorPos < 0)
		{
			menuCursorPos = menuLength - 1;
		}
		if(menuCursorPos > menuLength - 1)
		{
			menuCursorPos = 0;
		}
		for(int i = 0; i < std::end(menuPos) - std::begin(menuPos); i++)
		{
			display.setCursor(5, menuPos[i]);
			if(i == menuCursorPos)
			{
				display.setTextColor(MENU_SELECT);
			}
			else
			{
				display.setTextColor(MENU_UNSELECT);
			}
			display.print(menuString[i]);
		}
		
		while(1)
		{
			if(buttonUp.pressed())
			{
				menuCursorPos -= 1;
				break;
			}
			if(buttonDown.pressed())
			{
				menuCursorPos += 1;
				break;
			}
			if(buttonSelect.pressed())
			{
				delay(250);
				if(buttonSelect.read() == Button::PRESSED)
				{
					exitLoop = true;
					break;
				}
				else {
					bool exitBrightMenu = false;
					display.fillScreen(ST77XX_WHITE);
					display.setTextSize(2);
					display.setTextColor(ST77XX_BLACK, ST77XX_WHITE);
					display.setCursor(5, 5);
					display.print(menuString[menuCursorPos]);
					
					while(1)
					{
						if(menuCursorPos == 0) // user has selected "backlight brightness"
						{
							display.setCursor(5, 35);
							display.print("Brightness: ");
							display.print(tftBrightLevel);

							if(buttonSelect.pressed())
							{
								exitBrightMenu = true;
								break;
							}
							if(buttonUp.pressed() && tftBrightLevel < sizeof(tftBrightness)/sizeof(tftBrightness[0]) - 1)
							{
								tftBrightLevel += 1;
							}
							if(buttonDown.pressed() && tftBrightLevel > 0)
							{
								tftBrightLevel -= 1;
							}
							analogWrite(tftBacklight, tftBrightness[tftBrightLevel]);
						}
						else
						{
							break;
						}
					}
					return configurationMenu();
				}
			}
		}
		if(exitLoop == true)
		{
			break;
		}
	}
}

/*
Checks if the current drybulb temp is higher or lower than the min or max for the unit, and updates min/max with current drybulb temp if so
*/
void updateMinMaxDB(uint8_t unit)
{
	// if this is the first time we called this function, min and max values won't exist in json, create them
	if(j["units"][unit]["max_db"] == nullptr)
	{
		j["units"][unit]["max_db"] = j["units"][unit]["dry_bulb"];
	}
	if(j["units"][unit]["min_db"] == nullptr)
	{
		j["units"][unit]["min_db"] = j["units"][unit]["dry_bulb"];
	}

	// if we haven't created drybulb temp yet, we need to exit or we'll throw exeption on getting drybulb temp later
	if(j["units"][unit]["dry_bulb"] == nullptr)
	{
		return;
	}

	// check if current values are less or greater than min/max, update min/max if so.
	if(j["units"][unit]["dry_bulb"].template get<float>() > j["units"][unit]["max_db"].template get<float>())
	{
		j["units"][unit]["max_db"] = j["units"][unit]["dry_bulb"]; 
	}
	if(j["units"][unit]["dry_bulb"].template get<float>() < j["units"][unit]["min_db"].template get<float>())
	{
		j["units"][unit]["min_db"] = j["units"][unit]["dry_bulb"]; 
	}
}