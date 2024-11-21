/***************************************************
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

/* 
Sketch for indoor unit of indoor/outdoor "which is better" project for Chris.
Indoor unit has ThinkInk e-ink display & RTC with SD card logging & SHT45 RH/temp sensor.
Outdoor unit has RHT45 sensor and goes to sleep to save battery.
Both units are built on a M0 powered Feather with RFM69 915 MHz radio (license free).

Cut e-ink ECS pin, soldered wire to move it to pin A1 on feather (GPIO 15 on M0 Feather)
Cut SDCS pin on RTC FeatherWing, jumpered to pin A2 on feather (GPIO 16 on M0 Feather)

Current status is a stable build. 
  Has last update time for indoor & outdoor time displayed
  Logs inside and outside data in CSV format on SD card
    - Format is inside time, inside temp, inside RH, inside WBGT, outside time, outside temp, outside RH, outside WBGT, RSSI
  Added error message screen
  Added homemade message parser and conversion to floats
  2024-01-27: Added unit ID, display of keg temp if button C is pressed
  2024-02-18: Added parsing for unit 3, a AQI sensor. Really need to get the Raspberry Pi going...

Jonathan Seyfert
2024-02-18
*/

//#include "Adafruit_ThinkInk.h" // for e-ink display
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include "Adafruit_SHT4x.h" // for SHT45 temp/humidity sensor
#include <SPI.h>  // For RFM69 & LCD & SD card
#include <RH_RF69.h>  // For RFM69
#include <SD.h> // For SD card logging
#include "RTClib.h" // For RTC
#include "Button.h" //for buttons

// The following is for the e-ink display
// Note that the e-Ink Wing has an SD card CS on pin 5, do not use on accident!
// #define EPD_DC      10 // can be any pin, but required!
// #define EPD_CS      15  // pin 15 is A1 (19 on 32U4!)
// #define EPD_BUSY    -1  // can set to -1 to not use a pin (will wait a fixed delay) <- Available but not connected
// #define SRAM_CS     6  // can set to -1 to not use a pin (uses a lot of RAM!)
// #define EPD_RESET   -1  // can set to -1 and share with chip Reset (can't deep sleep)
#define TFT_CS        11
#define TFT_RST       12
#define TFT_DC        10
#define VBATPIN A7 // Internal battery voltage divider measurement pin
#define RTC_SD_CS 16 // RTC wing SD chip select pin
#define COLOR1 EPD_BLACK  // for ThinkInk Display
#define COLOR2 EPD_LIGHT  // for ThinkInk Display
#define COLOR3 EPD_DARK  // for ThinkInk Display
#define RF69_FREQ 915.0  // RFM69 frequency (MHz)
#define RFM69_CS      8  // RFM69 pins on M0 Feather
#define RFM69_INT     3  // RFM69 pins on M0 Feather
#define RFM69_RST     4  // RFM69 pins on M0 Feather
#define LED           13 // Built-in LED pin, also GPIO if sharing the LED is cool
#define SDCS          16 // CS for RTC datalogging wing SD card
// THE ABOVE IS DUPLICATED AND DEFINED AS RTC_SD_CS as well. Why?
#define SERIAL_DEBUG // Enables various serial debugging messages if defined

RTC_PCF8523 rtc; // for RTC
//Button buttonA(11); // Button A on e-Ink display I wonder if leaving these caused LCD issues...
//Button buttonB(12); // Button B on e-Ink display
//Button buttonC(13); // Button C on e-Ink display
Button buttonC(6); // labled C for legacy reasons, as e-ink was buttons A-C, need to rename now that not using e-ink

extern "C" char *sbrk(int i);  // for FreeMem()
const unsigned long updateTime = 5000;  // How often to update display
unsigned long timer = 0;  // Used to check if it's time to update display
const int radioSendTime = 15000;  // Send data via radio every 15 seconds
unsigned long logTimer = 0;
const int logTime = 60000;
unsigned long watchdogBlinkTimer = 0;
const int watchdogBlinkInterval = 1000;
int watchdogFadeTime = 100; // how often to adjust brightness
int ledBrightness = 255;
int fadeAmount = 10; //how much to adjust brightness
//char outDB[5], outWBGT[5], outRH[6], outVolt[5]; // for parsing outdoor sensor values from Rx packet
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"}; // for RTC
DateTime bootTime; // To display time at boot, time for min/max
DateTime outRxTime; // To display time outdoor sensor last updated
DateTime kegRxTime; // To display time keg sensor last updated
DateTime aqiRxTime; // To record time aqi sensor last updated
float indoorMax; // To record indoor max temp
float indoorMin; // To record indoor min temp
float outdoorMax; // to record outdoor max temp
float outdoorMin; // to record outdoor min temp
bool outdoorMinMaxInitialized = false;
const unsigned long minMaxDisplayTime = 15000; // How long to display the Min/Max screen
float outDB;
float outWBGT;
float outRH;
float outBat;
float kegDB;
float kegWBGT;
float kegRH;
float kegBat;
float aqiDB;
float aqiRH;
float aqiBat;
unsigned int aqi03um; // 0.3 um
unsigned int aqi05um;
unsigned int aqi10um;
unsigned int aqi25um;
unsigned int aqi50um;
unsigned int aqi100um; // 10 um
int outRSSI; // for RSSI of last out Rx
int kegRSSI;
int aqiRSSI;

Adafruit_SHT4x sht4 = Adafruit_SHT4x(); // for SHT45
//ThinkInk_290_Grayscale4_T5 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY); // Instance for ThinkInk FeatherWing
Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST); // for LCD
RH_RF69 rf69(RFM69_CS, RFM69_INT); // RFM69 Singleton instance

void setup() {
  #ifdef SERIAL_DEBUG
  Serial.begin(115200);  // for testing
  #endif

//  buttonA.begin(); // Initialize the button library function buttons
//  buttonB.begin();
  buttonC.begin();

  // pinMode(8, OUTPUT);  // When not using RFM, need to keep pin 8 high when sharing SPI bus
  // digitalWrite(8, HIGH);

  pinMode(RFM69_RST, OUTPUT);  // RFM69
  digitalWrite(RFM69_RST, LOW);  // RFM69
  pinMode(LED, OUTPUT); // Set LED pin to high
  analogWrite(LED, ledBrightness);

  // reset RFM69
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  rf69.init(); // Initialize RFM69 radio
  rf69.setFrequency(RF69_FREQ); // Set RFM69 radio frequency
  // Encryption key
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);  // Set RFM69 encryption
  SPI.usingInterrupt(digitalPinToInterrupt(RFM69_INT)); // Apparently the RadioHead library doesn't register it does SPI within an interrupt, so this is required to avoid conflicts
  
//  display.begin(THINKINK_GRAYSCALE4);  // Initialize e-Ink display
  display.init(170, 320);           // Init ST7789 170x320 LCD
  display.setRotation(1); // 0 & 2 should be portrait, 1 & 3 landscape
  display.fillScreen(ST77XX_WHITE);
  delay(5000);  // Pause for 5 seconds to allow sensor time to initialize before displaying inital values

  if(!sht4.begin()) { // initialize SHT45 sensor
    #ifdef SERIAL_DEBUG
    Serial.println(F("Unable to initialize SHT45!"));
    #endif
    displayError(F("Unable to initialize SHT45!"));
  }

  sht4.setPrecision(SHT4X_HIGH_PRECISION); // can use MED or LOW, HIGH takes longer
  sht4.setHeater(SHT4X_NO_HEATER); // can use 6 different heater options, see example
 
  // see if the card is present and can be initialized:
  if (!SD.begin(SDCS)) {
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

  rtc.start(); // In case RTC was stopped, this will start it
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

void loop() {
  if (rf69.available()) {
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf69.recv(buf, &len))
    {
      if (!len) return; // checks to see if message length is 0. Unclear why that's needed, since rf69.available should not return true if there is no message?
      buf[len] = 0; // what does this do?
      String rxPacket = ((char*)buf); // transfer message into String variable
      int id = parseRxPacket(rxPacket); // parses out received packet into individual float variables and checks/updates outdoor Min/Max temps
      if (id == 1) {
        outRxTime = rtc.now(); // Get time, so we can display time outside data was last received
        outRSSI = rf69.lastRssi();
      }
      else if (id == 2) { // kegerator sensor
        kegRxTime = rtc.now();
        kegRSSI = rf69.lastRssi();
      }

      #ifdef SERIAL_DEBUG
      Serial.println(rxPacket);
      #endif
    }
  }
  
  if(millis() - timer >= updateTime) // run every updateTime ms interval
  {
    sensors_event_t humidity, temp; // for SHT45
    sht4.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data
    float dryBulb = temp.temperature; // Get SHT45 temp
    if(dryBulb > indoorMax) { // update indoor Max temp if current temp is higher
      indoorMax = dryBulb;
    }
    if(dryBulb < indoorMin) { // update indoor Min temp if current temp is lower
      indoorMin = dryBulb;
    }
    float wetBulb = wetBulbCalc(dryBulb, humidity.relative_humidity); // calculate wet bulb temp
    updateDisplay(dryBulb*1.8 + 32, humidity.relative_humidity, wetBulb*1.8 + 32); // update display with current readings
    timer = millis(); // reset timer so this function is called at appropriate timing
  }

  if(millis() - logTimer >= logTime) { // log data every "logTime" ms interval
    sensors_event_t humidity, temp; // for SHT45
    sht4.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data
    float dryBulb = temp.temperature; // Get SHT45 temp
    float wetBulb = wetBulbCalc(dryBulb, humidity.relative_humidity); // calculate wet bulb temp
    logTempData(dryBulb*1.8 + 32, humidity.relative_humidity, wetBulb*1.8 + 32); // log the data to SD card
    logTimer = millis(); // reset timer so this function is called at appropriate timing
  }

  // if(buttonB.pressed()) { // display the Min/Max temps recorded when Button B on the e-Ink FeatherWing is pressed!
  //   displayMinMax();
  //   timer = millis() - updateTime + minMaxDisplayTime; //forces display update after minMaxDisplayTime
  // }

  if(buttonC.pressed()) { // display the Min/Max temps recorded when Button B on the e-Ink FeatherWing is pressed!
    displayKegData();
    timer = millis() - updateTime + minMaxDisplayTime; //forces display update after minMaxDisplayTime
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

void updateDisplay(float DB, float RH, float WB) {
//  display.clearBuffer();
  display.fillScreen(ST77XX_WHITE);
  display.setTextSize(2);
  display.setTextColor(ST77XX_BLACK);
  display.setCursor(5, 5);
  display.print(F("         Indoor  Outdoor"));
  
  display.setCursor(5, 30);
  display.print(F("Temp (F)  "));
  display.print(DB, 1);
  display.print(F("    "));
  display.print(outDB, 1);

  display.setCursor(5, 55);
  display.print(F("RH   (%)  "));
  display.print(RH, 1);
  display.print(F("    "));
  display.print(outRH, 1);

  display.setCursor(5, 80);
  display.setTextColor(ST77XX_BLACK);
  display.print(F("WBGT (F)  "));
  display.print(0.7*WB + 0.3*DB, 1);
  display.print(F("    "));
  display.print(outWBGT, 1);

  display.setCursor(5, 105);
  display.setTextColor(ST77XX_BLACK);
  display.print(F("Bat  (V)  "));
  display.print(batteryVoltage(), 2);  
  display.print(F("    "));
  display.print(outBat, 2);

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
  display.print(outRxTime.hour(), DEC);
  display.print(":");
  if (outRxTime.minute() < 10) {
    display.print("0");
  }
  display.print(outRxTime.minute(), DEC);

//  display.display();
}

void displayKegData() {
//  display.clearBuffer();
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

  delay(5000);
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
//  display.clearBuffer();
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

//  display.display();
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
//  display.clearBuffer();
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
//  display.display();

  while(!buttonC.pressed()) // stop on this screen until button C is pressed

  display.setCursor(5, 100);
  display.print(F("Button C pressed, continuing to proceed, ignoring error..."));
//  display.display();
  delay(5000); // pause for reading time before proceeding
}

float batteryVoltage() { // Measures and returns battery voltage
  float volts = analogRead(VBATPIN); // read battery voltage
  volts *= 2;    // we divided by 2, so multiply back
  volts *= 3.3;  // Multiply by 3.3V, our reference voltage
  volts /= 1024; // convert to voltage
  return volts;
}

/* Receives string as unitID/dryBulb/WBGT/humdity/battery voltage as X XX.X XX.X XX.X X.XX, parses to global floats, checks for outdoor Min/Max
    DOes not care about length of individual parts anymore, so long as they are space seperated
    Requires global float variables outDB, outWBGT, outRH, outBat, outdoorMax, outdoorMin
    Consider breaking out the Min/Max into it's own function? 
*/
int parseRxPacket(String rx) { 
  String parseString = "";
  int spaceCounter = 0;
  int length = rx.length(); // get string length
  int unitID = 0;
  #ifdef SERIAL_DEBUG
  Serial.println(length);
  #endif
  for (int i = 0; i < length; i++) {
    if (rx.charAt(i) != 32) { //check for space. ASCII decimal 32 is space
      parseString.concat(rx.charAt(i));
    }
    else {
      spaceCounter++;
      switch (spaceCounter) {
        case 1: // location of space x
          unitID = parseString.toInt();
          break;
        case 2: // location of space x
          if (unitID == 1) {
            outDB = parseString.toFloat();
          }
          else if (unitID == 2) {
            kegDB = parseString.toFloat();
          }
          break;
        case 3: // location of space x
          if (unitID == 1) {
            outWBGT = parseString.toFloat();
          }
          else if (unitID == 2) {
            kegWBGT = parseString.toFloat();
          }
          break;
        case 4: // location of space x
          if (unitID == 1) {
            outRH = parseString.toFloat();
          }
          else if (unitID == 2) {
            kegRH = parseString.toFloat();
          }
          break;
      }
      parseString = ""; // wipe string
    }
    if (i == length - 1) { // check if this was the last for() loop iteration
      if (unitID == 1) {
        outBat = parseString.toFloat();
      }
      else if (unitID == 2) {
        kegBat = parseString.toFloat();
      }
      else if (unitID == 3) {
        aqi100um = parseString.toDouble();
      }
    }
  }

  if (unitID == 1) { // only do this if we're talking about the outdoor unit
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