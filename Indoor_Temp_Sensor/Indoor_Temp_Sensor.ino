/*
Copyright 2024, 2025 Jonathan Seyfert

This file is part of Indoor-Outdoor_Temp_Sensor.

Indoor-Outdoor_Temp_Sensor is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License
as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

Indoor-Outdoor_Temp_Sensor is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Indoor-Outdoor_Temp_Sensor. If not, see <https://www.gnu.org/licenses/>. 
*/

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include "Adafruit_SHT4x.h" // for SHT45 temp/humidity sensor
#include <SPI.h>  // For RFM69 & LCD & SD card
#include <RH_RF69.h>  // For RFM69
#include <RHReliableDatagram.h> // FOr RFM69
#include <SD.h> // For SD card logging
#include "RTClib.h" // For RTC
#include "Button.h" //for buttons
#include <array>
#include <json.hpp>
//Must place json.hpp this in your Arduino IDE libraries folder in a folder called "json". 
//When including in sketch, it must come after including RH_RF69.h or you must include Arduino.h AFTER this library or compiling will fail due to how Arduino defines abs() with a macro

//#define VBATPIN       A7 // 9/A7 - Internal battery voltage divider measurement pin
const int VBATPIN = A7;  // 9/A7 - Internal battery voltage divider measurement pin
#define RF69_FREQ     915.0  // RFM69 frequency (MHz)
#define RFM69_CS      8  // RFM69 pins on M0 Feather
#define RFM69_INT     3  // RFM69 pins on M0 Feather
#define RFM69_RST     4  // RFM69 pins on M0 Feather
#define BUTTON_DOWN   5  // Down button pin
#define BUTTON_UP     6  // Up button pin
#define BUTTON_SELECT 14 // (A0) Select button pin 
#define TFT_BACKLIGHT 15 // (A1) pin for backlight PWM
#define TFT_DC        10 // DC for LCD display
#define TFT_CS        11 // Chip select for LCD display
#define TFT_RST       12 // Reset for LCD display
#define LED           13 // Built-in LED pin, also GPIO if sharing the LED is cool
#define SDCS          16 // CS for RTC datalogging wing SD card
#define SERIAL_DEBUG     // Enables various serial debugging messages if defined
#define MENU_SELECT ST77XX_WHITE, ST77XX_BLACK
#define MENU_UNSELECT ST77XX_BLACK, ST77XX_WHITE

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

extern "C" char *sbrk(int i);  // for FreeMem()
const unsigned long updateTime = 5000;  // How often to update display
unsigned long timer = 0;  // Used to check if it's time to update display
const int radioSendTime = 15000;  // Send data via radio every 15 seconds
unsigned long logTimer = 0;
const int logTime = 60000;
unsigned long watchdogBlinkTimer = 0;
const int watchdogBlinkInterval = 1000;
int watchdogFadeTime = 100; // how often to adjust brightness
int ledBrightness = 255; // brightness of the built-in LED
int fadeAmount = 10; //how much to adjust brightness
const int tftBrightness[] = {1, 10, 40, 100, 255}; // brightness of the TFT display backlight
int tftBrightLevel = 4;
const int altDisplayTime = 10000; // time to display alternate screens (min/max, alternate sensor)
//char outDB[5], outWBGT[5], outRH[6], outVolt[5]; // for parsing outdoor sensor values from Rx packet
const char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"}; // for RTC
DateTime bootTime; // To display time at boot, time for min/max
DateTime outRxTime; // To display time outdoor sensor last updated
DateTime kegRxTime; // To display time keg sensor last updated
DateTime aqiRxTime; // To record time aqi sensor last updated
float indoorMax, indoorMin; // To record indoor max/min temp
float outdoorMax, outdoorMin; // To record outdoor max/min temp
bool outdoorMinMaxInitialized = false;
const unsigned long minMaxDisplayTime = 15000; // How long to display the Min/Max screen
float outDB, outWBGT, outRH, outBat; // to store outside sensor values
float kegDB, kegWBGT, kegRH, kegBat; // to store kegerator sensor values
float aqiDB, aqiRH, aqiBat; // to store AQI values. Probably remove, this was a one-time deal, and future AQI will be integrated into the "out" or other sensors as appropriate
unsigned int aqi03um, aqi05um, aqi10um, aqi25um, aqi50um, aqi100um; // 0.3-10 um concentration values
int outRSSI, kegRSSI, aqiRSSI; // for RSSI of remote sensors
uint8_t encryptionKey[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // encryption key for RMF69 radio. Must match that on remote sensor!
                  0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}; // should change the default encryption key...
String rxData;

enum Addresses {Base = 0, Outdoor = 1, Basement = 2,}; // addresses for all units in network
nlohmann::json j; // one json to hold all variables from all units

//nlohmann::json j2; // json to hold data from unit 2
/*
Dealing with json data:
If you want a String of a specific item, then use:
  String(j2["wet_bulb"].dump().c_str())

Similarily, if you wanted to do a numerical comparison on a value in the json, you could do:
  if (String(j2["wet_bulb"].dump().c_str()).toFloat() > 42)

Kinda clunky, but I'm still thinking that OVERALL using the json will be better for handling recieved data.

No, actually you can do:
  if (j2["wet_bulb"] != nullptr)
  {
    auto number = j2["wet_bulb"].template get<double>();
    Serial.println(number);
  }
Just need to make sure there's a value one way or another (check for nullptr or verify you wrote the data first)

Can update a value (in j2) with:
  nlohmann::json j;
  j["wet_bulb"] = 42.42;
  j2.update(j);
*/

//RTC_PCF8523 rtc; // for RTC
RTC_DS3231 rtc; // for RTC
Button buttonUp(BUTTON_UP); // for an up button. Defaults to 100 ms debounch, use (pin, debounce_ms) to change
Button buttonDown(BUTTON_DOWN); // for a down button
Button buttonSelect(BUTTON_SELECT); // for a down button
Adafruit_SHT4x sht4 = Adafruit_SHT4x(); // for SHT45
Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST); // for LCD
RH_RF69 driver(RFM69_CS, RFM69_INT); // Singleton instance of the radio driver
RHReliableDatagram manager(driver, Base); // Class to manage message delivery and receipt, using the driver declared above

void setup() {
  #ifdef SERIAL_DEBUG
  Serial.begin(115200);  // for testing
  #endif

  // Initialize the buttons using the button library
  buttonUp.begin();
  buttonDown.begin();
  buttonSelect.begin();

  pinMode(VBATPIN, INPUT);
  // When not using RFM, uncomment the below two lines to keep pin 8 RFM69 CS high, otherwise RMF69 will conflict with other SPI devices
  // pinMode(8, OUTPUT);  
  // digitalWrite(8, HIGH);

  pinMode(LED, OUTPUT); // Configure LED pin as output
  analogWrite(LED, ledBrightness); // Give LED an initial value
  pinMode(TFT_BACKLIGHT, OUTPUT);
  analogWrite(TFT_BACKLIGHT, tftBrightness[tftBrightLevel]); // set lcd to full brightness to start

  // reset RFM69
  pinMode(RFM69_RST, OUTPUT);  
  digitalWrite(RFM69_RST, LOW);  // is this needed?
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  manager.init(); // Initialize RFM69 radio
  driver.setFrequency(RF69_FREQ); // Set RFM69 radio frequency
  driver.setTxPower(20, true); // Power levels can be set from -2 to 20, 2nd arg must be true for RFM69. 1% duty cycle at 20, VSWR 3:1 max
  driver.setEncryptionKey(encryptionKey);  // Set RFM69 encryption key
  SPI.usingInterrupt(digitalPinToInterrupt(RFM69_INT)); // The RadioHead library doesn't register it does SPI within an interrupt, so this is required to avoid conflicts
  
  display.init(170, 320); // Initialize the LCD ST7789 (170x320 pixels)
  display.setRotation(3); // 0 & 2 are portrait, 1 & 3 are landscape
  display.fillScreen(ST77XX_WHITE); // fill screen with white because we draw with black
  delay(100);  // Give the screen a sec to initialize

  if(!sht4.begin()) { // initialize SHT45 sensor
    #ifdef SERIAL_DEBUG
    Serial.println(F("Unable to initialize SHT45!"));
    #endif
    displayError(F("Unable to initialize SHT45!"));
  }
  sht4.setPrecision(SHT4X_HIGH_PRECISION); // can use MED or LOW, HIGH takes longer
  sht4.setHeater(SHT4X_NO_HEATER); // can use 6 different heater options, see example
 
  if (!SD.begin(SDCS)) { // initialize the SD card
    #ifdef SERIAL_DEBUG
    Serial.println(F("Card init. failed!"));
    #endif
    displayError(F("Unable to initialize SD card!"));
  }

  if(!rtc.begin()) { // initialize RTC
    #ifdef SERIAL_DEBUG
    Serial.println(F("RTC init. failed!")); // print error over USB Serial port
    #endif
    displayError(F("Unable to initialize RTC!")); // display error on screen
  }
  //rtc.start(); // In case RTC was stopped, this will start it
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // adjusts date and time to that of sketch compiling. Comment out after first upload
  bootTime = rtc.now(); // Record the DateTime of boot

  // The below updates the display once on power-up
  sensors_event_t humidity, temp; // for SHT45
  sht4.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data
  float dryBulb = temp.temperature; // Get indoor temp
  indoorMin = dryBulb; //set indoor min to current for new boot
  indoorMax = dryBulb; //set indoor max to current for new boot
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
    indoor["battery_voltage"] = round(analogRead(VBATPIN)*0.006445*100)/100; // ADC*2*3.3/1024 *2 - 50% voltage divider. *3.3 - V_ref. /1024 - 8-bit ADC. Round 2 decimal places
    updateJson(indoor, 0);

    // float dryBulb = temp.temperature; // Get SHT45 temp
    // if(dryBulb > indoorMax) // update indoor Max temp if current temp is higher
    //   indoorMax = dryBulb;
    // if(dryBulb < indoorMin) // update indoor Min temp if current temp is lower
    //   indoorMin = dryBulb;
    // float wetBulb = wetBulbCalc(dryBulb, humidity.relative_humidity); // calculate wet bulb temp
    //updateDisplay(dryBulb*1.8 + 32, humidity.relative_humidity, wetBulb*1.8 + 32); // update display with current readings
    updateDisplay((j["units"][0]["dry_bulb"].template get<float>())*1.8 + 32, j["units"][0]["humidity"].template get<float>(), (j["units"][0]["wet_bulb"].template get<float>())*1.8 + 32); // update display with current readings
    timer = millis(); // reset timer so this function is called at appropriate timing 

    // if (j["units"][1]["wet_bulb"] != nullptr)
    // {
    //   float number = j["units"][1]["wet_bulb"].template get<float>();
    //   Serial.println(number);
    // }
  }

  if(millis() - logTimer >= logTime) { // log data every "logTime" ms interval
    sensors_event_t humidity, temp; // for SHT45
    sht4.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data
    float dryBulb = temp.temperature; // Get SHT45 temp
    float wetBulb = wetBulbCalc(dryBulb, humidity.relative_humidity); // calculate wet bulb temp
    logTempData(dryBulb*1.8 + 32, humidity.relative_humidity, wetBulb*1.8 + 32); // log the data to SD card
    logTimer = millis(); // reset timer so this function is called at appropriate timing
  }

  // if(buttonUp.pressed()) { // display the Min/Max temps recorded when Button B on the e-Ink FeatherWing is pressed!
  //   delay(250);
  //   if (buttonUp.read() == Button::PRESSED) {
  //     configurationMenu();
  //     //displayKegData();
  //     //delay(2500);
  //   }
  //   else {
  //     displayMinMax();
  //     delay(2500);
  //   }
  // }
  if(buttonUp.pressed()) {
    displayMinMax();
    delay(altDisplayTime);
  }
  if(buttonSelect.pressed()) {
    Serial.println("opening config menu from main loop");
    configurationMenu();
  }
  if(buttonDown.pressed()) {
    displayKegData();
    delay(altDisplayTime);
  }

  if(millis() - watchdogBlinkTimer >= watchdogFadeTime) {
    ledBrightness += fadeAmount;
    if (ledBrightness <= 0) {
      ledBrightness = 0;
      fadeAmount = -fadeAmount;
    }
    if (ledBrightness >= 255) {
      ledBrightness = 255;
      fadeAmount = -fadeAmount;
    }
    analogWrite(LED, ledBrightness);
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

    nlohmann::json jRx = nlohmann::json::parse(rxData); // temp json object to hold received data
    jRx["RSSI"] = driver.lastRssi();
    rxData = ""; // wipe string
    return updateJson(jRx, from);
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
  display.print(batteryVoltage(), 2);  
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

void displayKegData() {
  display.fillScreen(ST77XX_WHITE);
  display.setTextSize(2);
  display.setTextColor(ST77XX_BLACK);
  display.setCursor(5, 5);
  display.print(F("          Keg"));
  
  display.setCursor(5, 30);
  display.print(F("Temp (F)  "));
  display.print(kegDB, 1);

  display.setCursor(5, 55);
  display.print(F("RH   (%)  "));
  display.print(kegRH, 1);

  display.setCursor(5, 80);
  display.setTextColor(ST77XX_BLACK);
  display.print(F("WBGT (F)  "));
  display.print(kegWBGT, 1);

  display.setCursor(5, 105);
  display.setTextColor(ST77XX_BLACK);
  display.print(F("Bat  (V)  "));
  display.print(kegBat, 2);

  display.drawFastVLine(195, 0, 128, ST77XX_BLACK);
  display.drawFastVLine(105, 0, 128, ST77XX_BLACK);
  display.drawFastHLine(0, 25, 296, ST77XX_BLACK);

  DateTime now = rtc.now(); // Get the current time to update the display with
  display.setTextSize(1);
  display.setTextColor(ST77XX_BLACK);
  display.setCursor(5, 4);
  display.print(F("Last update:"));
  display.setCursor(5, 14);
  display.print("I-"); // indoor display update time
  if (now.hour() < 10) {
    display.print("0");
  }
  display.print(now.hour(), DEC);
  display.print(":");
  if (now.minute() < 10) {
    display.print("0");
  }
  display.print(now.minute(), DEC);
  display.print("  O-"); // outdoor display update time
  if (outRxTime.hour() < 10) {
    display.print("0");
  }
  display.print(kegRxTime.hour(), DEC);
  display.print(":");
  if (outRxTime.minute() < 10) {
    display.print("0");
  }
  display.print(kegRxTime.minute(), DEC);
}

float wetBulbCalc(float DB, float RH){
  float WBC = DB*(atan(0.151977*pow(RH + 8.313659,0.5))) + atan(DB + RH) - atan(RH - 1.676331) + 0.00391838*pow(RH, 1.5)*atan(0.023101*RH) - 4.686035;
  return WBC;
}

int FreeMem () {  //http://forum.arduino.cc/index.php?topic=365830.msg2542879#msg2542879
  char stack_dummy = 0;
  return &stack_dummy - sbrk(0);
}

void displayMinMax() {
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
  display.print(indoorMin*1.8 + 32);
  display.print(F("/"));
  display.print(indoorMax*1.8 + 32);

  display.setCursor(5, 70);
  display.print(F("Outdoor: "));
  display.print(outdoorMin);
  display.print(F("/"));
  display.print(outdoorMax);

}

void logTempData(float DB, float RH, float WB) {
  String logString = "";
  DateTime logTime = rtc.now(); // variable for logging time of log
  logString += logTime.year();
  logString += "-";
  logString += logTime.month();
  logString += "-";
  logString += logTime.day();
  logString += " ";
  if (logTime.hour() < 10) { // add a leading zero to keep conventional time format
    logString += "0";
  }
  logString += logTime.hour();
  logString += ":";
  if (logTime.minute() < 10) {
    logString += "0";
  }
  logString += logTime.minute();
  logString += ":";
  if (logTime.second() < 10) {
    logString += "0";
  }
  logString += logTime.second();
  logString += ",";
  logString += DB;
  logString += ",";
  logString += RH;
  logString += ",";
  logString += 0.7*WB + 0.3*DB;
  logString += ",";
  logString += batteryVoltage();
  logString += ",";
  logString += outRxTime.year();
  logString += "-";
  logString += outRxTime.month();
  logString += "-";
  logString += outRxTime.day();
  logString += " ";
  if (outRxTime.hour() < 10) { // add a leading zero to keep conventional time format
    logString += "0";
  }
  logString += outRxTime.hour();
  logString += ":";
  if (outRxTime.minute() < 10) { // add a leading zero to keep conventional time format
    logString += "0";
  }
  logString += outRxTime.minute();
  logString += ":";
  if (outRxTime.second() < 10) { // add a leading zero to keep conventional time format
    logString += "0";
  }
  logString += outRxTime.second();
  logString += ",";
  logString += outDB;
  logString += ",";
  logString += outRH;
  logString += ",";
  logString += outWBGT;
  logString += ",";
  logString += outBat; // log battery voltage
  logString += ",";
  logString += outRSSI;
  logString += ",";
  logString += kegRxTime.year();
  logString += "-";
  logString += kegRxTime.month();
  logString += "-";
  logString += kegRxTime.day();
  logString += " ";
  if (kegRxTime.hour() < 10) { // add a leading zero to keep conventional time format
    logString += "0";
  }
  logString += kegRxTime.hour();
  logString += ":";
  if (kegRxTime.minute() < 10) { // add a leading zero to keep conventional time format
    logString += "0";
  }
  logString += kegRxTime.minute();
  logString += ":";
  if (kegRxTime.second() < 10) { // add a leading zero to keep conventional time format
    logString += "0";
  }
  logString += kegRxTime.second();
  logString += ",";
  logString += kegDB;
  logString += ",";
  logString += kegRH;
  logString += ",";
  logString += kegWBGT;
  logString += ",";
  logString += kegBat; // log battery voltage
  logString += ",";
  logString += kegRSSI;

  File logFile = SD.open("datalog", FILE_WRITE); // Open file for logging crash as writable file
  if(logFile) {
    logFile.println(logString); // print to file
    logFile.close(); // close file to ensure it was written
    #ifdef SERIAL_DEBUG
    Serial.println(F("Logged to datalog.txt!")); // for debug
    #endif
  }
  else {
    #ifdef SERIAL_DEBUG
    Serial.println(F("Unable to open datalog.txt!")); // for debug
    #endif
    //displayError(F("Unable to open datalog.txt!")); // display error
  }
  #ifdef SERIAL_DEBUG
  Serial.println(logString); // for debug
  #endif
}

void displayError(String error) {
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

float batteryVoltage() { // Measures and returns battery voltage
  float volts = analogRead(VBATPIN); // read battery voltage
  volts *= 2;    // we divided by 2, so multiply back
  volts *= 3.3;  // Multiply by 3.3V, our reference voltage
  volts /= 1024; // convert to voltage
  return volts;
}

/* Receives string as dryBulb/WBGT/humdity/battery voltage as XX.X XX.X XX.X X.XX, parses to global floats, checks for outdoor Min/Max
    DOes not care about length of individual parts anymore, so long as they are space seperated
    Requires global float variables outDB, outWBGT, outRH, outBat, outdoorMax, outdoorMin
    Consider breaking out the Min/Max into it's own function? 
*/
int parseRxPacket(String rx, uint8_t unitID) { 
  String parseString = "";
  int spaceCounter = 0;
  int length = rx.length(); // get string length
  for (int i = 0; i < length; i++) {
    if (rx.charAt(i) != 32) { //check for space. ASCII decimal 32 is space
      parseString.concat(rx.charAt(i));
    }
    else {
      spaceCounter++;
      switch (spaceCounter) {
        case 1: // location of space x
          if (unitID == 2) {
            outDB = parseString.toFloat();
          }
          else if (unitID == 3) {
            kegDB = parseString.toFloat();
          }
          break;
        case 2: // location of space x
          if (unitID == 2) {
            outWBGT = parseString.toFloat();
          }
          else if (unitID == 3) {
            kegWBGT = parseString.toFloat();
          }
          break;
        case 3: // location of space x
          if (unitID == 2) {
            outRH = parseString.toFloat();
          }
          else if (unitID == 3) {
            kegRH = parseString.toFloat();
          }
          break;
      }
      parseString = ""; // wipe string
    }
    if (i == length - 1) { // check if this was the last for() loop iteration
      if (unitID == 2) {
        outBat = parseString.toFloat();
      }
      else if (unitID == 3) {
        kegBat = parseString.toFloat();
      }
      else if (unitID == 4) {
        aqi100um = parseString.toDouble();
      }
    }
  }

  if (unitID == 2) { // only do this if we're talking about the outdoor unit
    if (outdoorMinMaxInitialized == false) { // see if we've ever done the initializing of outdoor Min/Max, if not, do so
      outdoorMax = outDB;
      outdoorMin = outDB;
      outdoorMinMaxInitialized = true; // set flag so we never do it again.
    }
    if(outDB > outdoorMax) {
      outdoorMax = outDB;
    }
    if(outDB < outdoorMin) {
      outdoorMin = outDB;
    }
  }
  
  return unitID;
}

void configurationMenu() {
  Serial.println("Top of config menu");
  display.fillScreen(ST77XX_WHITE);
  display.setTextSize(2);
  display.setTextColor(ST77XX_BLACK);
  display.setCursor(5, 5);
  display.print(F("Configuration Menu"));

  int menuCursorPos = 0; // what menu list option should be highlighted with cursor?
  int menuPos[] = {35, 55, 75}; // y-position values for menu entries
  String menuString[] = {"Backlight brightness", // list of menu entries
                         "Other setting       ",
                         "Other setting 2     "};

  while(1) {
    bool exitLoop = false;
    int menuLength = sizeof(menuPos)/sizeof(menuPos[0]);
    if(menuCursorPos < 0) {
      menuCursorPos = menuLength - 1;
    }
    if(menuCursorPos > menuLength - 1) {
      menuCursorPos = 0;
    }
    for(int i = 0; i < std::end(menuPos) - std::begin(menuPos); i++) {
      display.setCursor(5, menuPos[i]);
      if(i == menuCursorPos) {
        display.setTextColor(MENU_SELECT);
      }
      else {
        display.setTextColor(MENU_UNSELECT);
      }
      display.print(menuString[i]);
    }
    
    while(1) {
      if(buttonUp.pressed()) {
        menuCursorPos -= 1;
        break;
      }
      if(buttonDown.pressed()) {
        menuCursorPos += 1;
        break;
      }
      if(buttonSelect.pressed()) {
        delay(250);
        if(buttonSelect.read() == Button::PRESSED) {
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
          
          while(1) {
            if(menuCursorPos == 0) { // user has selected "backlight brightness"
              display.setCursor(5, 35);
              display.print("Brightness: ");
              display.print(tftBrightLevel);

              if(buttonSelect.pressed()) {
                exitBrightMenu = true;
                break;
              }
              if(buttonUp.pressed() & tftBrightLevel < 4) {
                tftBrightLevel += 1;
              }
              if(buttonDown.pressed() && tftBrightLevel > 0) {
                tftBrightLevel -= 1;
              }
              analogWrite(TFT_BACKLIGHT, tftBrightness[tftBrightLevel]);
            }
            else
              break;
          }
          return configurationMenu();
        }
      }
    }
    if(exitLoop == true) {
      break;
    }
  }
}