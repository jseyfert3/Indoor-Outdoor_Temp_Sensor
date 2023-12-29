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

Current status is a stable build that doesn't hang, though it's still lacking the last update
message to show if it's working or not (given the e-ink screen doesn't shut off when mCU stops running)

Jonathan Seyfert
2024-12-28
*/

#include "Adafruit_ThinkInk.h" // for e-ink display
#include "Adafruit_SHT4x.h" // for SHT45 temp/humidity sensor
#include <SPI.h>  // For RFM69 & e-ink & SD card
#include <RH_RF69.h>  // For RFM69
#include <SD.h> // For SD card logging

// The following is for the e-ink display
// Note that the e-Ink Wing has an SD card CS on pin 5, do not use on accident!
#define EPD_DC      10 // can be any pin, but required!
#define EPD_CS      15  // pin 15 is A1 (19 on 32U4!)
#define EPD_BUSY    -1  // can set to -1 to not use a pin (will wait a fixed delay) <- Available but not connected
#define SRAM_CS     6  // can set to -1 to not use a pin (uses a lot of RAM!)
#define EPD_RESET   -1  // can set to -1 and share with chip Reset (can't deep sleep)
#define BTN_A       11  // Button A on e-Ink
#define BTN_B       12  // Button B on e-Ink
#define BTN_C       13  // Button C on e-Ink
#define VBATPIN A7 // Internal battery voltage divider measurement pin
#define RTC_SD_CS 16 // RTC wing SD chip select pin
#define COLOR1 EPD_BLACK  // for ThinkInk Display
#define COLOR2 EPD_LIGHT  // for ThinkInk Display
#define COLOR3 EPD_DARK  // for ThinkInk Display
#define RF69_FREQ 915.0  // RFM69 frequency (MHz)
#define RFM69_CS      8  // RFM69 pins on M0 Feather
#define RFM69_INT     3  // RFM69 pins on M0 Feather
#define RFM69_RST     4  // RFM69 pins on M0 Feather
#define LED           13 // RFM69 pins on M0 Feather
#define SDCS          16 // CS for RTC datalogging wing SD card

File logfile; // name to use for file object

extern "C" char *sbrk(int i);  // for FreeMem()
const unsigned long updateTime = 180000;  // How often to update display
unsigned long timer = 0;  // Used to check if it's time to update display
const int radioSendTime = 15000;  // Send data via radio every 15 seconds
float batteryVoltage = 0; // for measuring battery voltage
String rxPacket;
char outDB[5], outWBGT[5], outRH[6], outVolt[5]; // for parsing outdoor sensor values from Rx packet

Adafruit_SHT4x sht4 = Adafruit_SHT4x(); // for SHT45
ThinkInk_290_Grayscale4_T5 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY); // Instance for ThinkInk FeatherWing
RH_RF69 rf69(RFM69_CS, RFM69_INT); // RFM69 Singleton instance

void setup()
{
  Serial.begin(115200);  // for testing

  // pinMode(8, OUTPUT);  // When not using RFM, need to keep pin 8 high when sharing SPI bus
  // digitalWrite(8, HIGH);

  pinMode(RFM69_RST, OUTPUT);  // RFM69
  digitalWrite(RFM69_RST, LOW);  // RFM69

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

  sht4.begin(); // initialize SHT45 sensor
  sht4.setPrecision(SHT4X_HIGH_PRECISION); // can use MED or LOW, HIGH takes longer
  sht4.setHeater(SHT4X_NO_HEATER); // can use 6 different heater options, see example
  
  display.begin(THINKINK_GRAYSCALE4);  // Initialize e-Ink display
  delay(5000);  // Pause for 5 seconds to allow sensor time to initialize before displaying inital values

  // see if the card is present and can be initialized:
  if (!SD.begin(SDCS)) {
    Serial.println("Card init. failed!");
    while(1);
  }

  logfile = SD.open("crashLog", FILE_WRITE); // Open file for logging crash as writable file

  // The below updates the display once on power-up
  sensors_event_t humidity, temp; // for SHT45
  sht4.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data
  float dryBulb = temp.temperature; // Convert SHT45 temp from C to F
  float wetBulb = wetBulbCalc(dryBulb, humidity.relative_humidity);
  readAndDisplaySCD30(dryBulb*1.8 + 32, humidity.relative_humidity, wetBulb*1.8 + 32, "Waiting...");

  SPI.usingInterrupt(digitalPinToInterrupt(RFM69_INT)); // Apparently the RadioHead library doesn't register it does SPI within an interrupt, so this is required to avoid conflicts
}

void loop() {
   if (rf69.available()) {
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf69.recv(buf, &len))
    {
      if (!len) return; //I think this is checking for buffer overflow? Part of example code
      buf[len] = 0;
      Serial.print("Received [");
      Serial.print(len);
      Serial.print("]: ");
      rxPacket = ((char*)buf);
      Serial.println(rxPacket); // rx format is dryBulb/wetBulb/humdity/battery voltage as XX.X XX.X XX.X X.XX
      Serial.print("RSSI: ");
      Serial.println(rf69.lastRssi(), DEC);
    }
   }
  
  if(millis() - timer >= updateTime)
  {
    sensors_event_t humidity, temp; // for SHT45
    sht4.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data
    float dryBulb = temp.temperature; // Get SHT45 temp
    float wetBulb = wetBulbCalc(dryBulb, humidity.relative_humidity);
    readAndDisplaySCD30(dryBulb*1.8 + 32, humidity.relative_humidity, wetBulb*1.8 + 32, rxPacket);
    timer = millis();
  }
}

void readAndDisplaySCD30(float DB, float RH, float WB, String RX)
{
  sscanf(RX.c_str(), "%s %s %s %s", &outDB, &outWBGT, &outRH, &outVolt); // parse packet to values
  batteryVoltage = analogRead(VBATPIN); // read battery voltage
  batteryVoltage *= 2;    // we divided by 2, so multiply back
  batteryVoltage *= 3.3;  // Multiply by 3.3V, our reference voltage
  batteryVoltage /= 1024; // convert to voltage

  display.clearBuffer();
  display.setTextSize(2);
  display.setTextColor(EPD_DARK);
  display.setCursor(5, 5);
  display.print(F("         Indoor  Outdoor"));
  
  display.setCursor(5, 30);
  display.print(F("Temp (F)  "));
  display.print(DB, 1);
  display.print(F("    "));
  display.print(outDB);

  display.setCursor(5, 55);
  display.print(F("RH   (%)  "));
  display.print(RH, 1);
  display.print(F("    "));
  display.print(outRH);

  display.setCursor(5, 80);
  display.setTextColor(EPD_BLACK);
  display.print(F("WBGT (F)  "));
  display.print(0.7*WB + 0.3*DB, 1);
  display.print(F("    "));
  display.print(outWBGT);

  display.setCursor(5, 105);
  display.setTextColor(EPD_DARK);
  display.print(F("Bat  (V)  "));
  display.print(batteryVoltage, 2);  
  display.print(F("    "));
  display.print(outVolt);

  display.drawFastVLine(195, 0, 128, EPD_BLACK);
  display.drawFastVLine(105, 0, 128, EPD_BLACK);
  display.drawFastHLine(0, 25, 296, EPD_BLACK);

  display.setTextSize(1);
  display.setTextColor(EPD_BLACK);
  display.setCursor(5, 4);
  display.print(F("Last update:"));
  display.setCursor(5, 14);
  display.print(F("HH:MM"));

  display.display();
}

float wetBulbCalc(float DB, float RH){
  float WBC = DB*(atan(0.151977*pow(RH + 8.313659,0.5))) + atan(DB + RH) - atan(RH - 1.676331) + 0.00391838*pow(RH, 1.5)*atan(0.023101*RH) - 4.686035;
  return WBC;
}

int FreeMem () {  //http://forum.arduino.cc/index.php?topic=365830.msg2542879#msg2542879
  char stack_dummy = 0;
  return &stack_dummy - sbrk(0);
}
