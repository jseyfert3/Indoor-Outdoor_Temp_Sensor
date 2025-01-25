/*
Sketch for outdoor unit of indoor/outdoor "which is better" project to determine if opening windows or keeping them closed feels better.
Indoor unit has graphic LCD display & RTC with SD card logging & SHT45 RH/temp sensor.
Outdoor unit has RHT45 sensor and goes to sleep to save battery.
Both units are built on a M0 powered Feather with RFM69 915 MHz radio (USA ISM frequency).

2024-01-27: Added Unit ID to transmission for multiple remote units
 
Cut e-ink ECS pin, soldered wire to move it to pin A1 on feather (GPIO 15 on M0 Feather)
Cut SDCS pin on RTC FeatherWing, jumpered to pin A2 on feather (GPIO 16 on M0 Feather)


Copyright 2024 Jonathan Seyfert

This file is part of Indoor-Outdoor_Temp_Sensor.

Indoor-Outdoor_Temp_Sensor is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License
as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

Indoor-Outdoor_Temp_Sensor is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Foobar. If not, see <https://www.gnu.org/licenses/>. 
*/

#include <SPI.h>  // For RFM69
#include <RH_RF69.h> // For RFM69
#include <Adafruit_SHT4x.h> // for SHT45 temp/humidity sensor
#include <Adafruit_SleepyDog.h> // for watchdog sleep for power savings

#define RF69_FREQ 915.0  // Frequency of RFM69
#define RFM69_CS      8  // RFM69 pins on M0 Feather
#define RFM69_INT     3  // RFM69 pins on M0 Feather
#define RFM69_RST     4  // RFM69 pins on M0 Feather
#define LED           13 // Built-in LED on M0 Feather
#define VBATPIN       A7 // Internal battery voltage divider measurement pin
#define SHTPWRPIN     5

//#define SERIAL_DEBUG // comment out if serial debugging is not desired. Watchdog sleep will be disabled if defined

Adafruit_SHT4x sht4 = Adafruit_SHT4x(); // for SHT45
RH_RF69 rf69(RFM69_CS, RFM69_INT); // Singleton instance of the radio driver

float x = 0.0; // for custom dtostrf()
float y = 0.0; // for custom dtostrf()
float batteryVoltage = 0; // for measuring battery voltage
char unitID[2] = {'1'}; // for applications with multiple sensors, set this to desired unit ID before compiling
// Unit 1 is outdoor. Unit 2 is kegerator.
// SHOULD ADD: Some dip switches that allow setting unitID instead of needing it to be compiled into code

void setup() {
  #ifdef SERIAL_DEBUG
  Serial.begin(115200);
  #endif

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  // manual reset RFM69
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  pinMode(SHTPWRPIN, OUTPUT); // Initialize pin for SHT45 power control
  digitalWrite(SHTPWRPIN, HIGH); //turn on SHT45 for initialization
  delay(100);
  sht4.begin(); // initialize SHT45 sensor
  sht4.setPrecision(SHT4X_HIGH_PRECISION); // can use MED or LOW, HIGH takes longer
  sht4.setHeater(SHT4X_NO_HEATER); // can use 6 different heater options, see example
  
  rf69.init(); // Initialize RFM69
  rf69.setFrequency(RF69_FREQ); // Set RFM69 frequency
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW
  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
}

void loop() {
  digitalWrite(LED, HIGH); // turn on LED to indicate not sleeping
  float dryBulb = 0.00;
  float wetBulb = 0.00;
  unsigned long tempTime = 0; // mostly for troubleshooting right now
  unsigned long rhTime = 0; // mostly for troubleshooting right now
  bool eventGood = false;
  sensors_event_t humidity, temp; // for SHT45

  for(int i = 1; i <= 10; i++) {
    digitalWrite(SHTPWRPIN, HIGH); // turn on SHT45
    delay(100); // give it some time to powerup
    eventGood = sht4.getEvent(&humidity, &temp); // request temp and humidity
    if(eventGood) { // check if our request for temp and humidity worked
      break;
    }
    else {
      digitalWrite(SHTPWRPIN, LOW);
      delay(50);

      #ifdef SERIAL_DEBUG
      Serial.print("failed to get event #");
      Serial.println(i);
      #endif
    }
  }

  dryBulb = temp.temperature;
  wetBulb = wetBulbCalc(dryBulb, humidity.relative_humidity);
  batteryVoltage = analogRead(VBATPIN); // read battery voltage
  batteryVoltage *= 0.006445;    // ADC*2*3.3/1024 (Multiply ADC value by 2 because of 50% voltage divider, multiply by 3.3 for V_ref, divide by 1024 for 8-bit ADC)

  String radioPacketString = "";
  if(!eventGood) { 
    radioPacketString = String(unitID) + " ---- ---- ---- " + String(batteryVoltage, 2); // we don't have new data, so don't send new data
  }
  else {
    radioPacketString = String(unitID) + " " + String(dryBulb*1.8 + 32, 1) + " " + String((0.7*wetBulb + 0.3*dryBulb)*1.8 + 32, 1) + " "
                                       + String(humidity.relative_humidity, 1) + " " + String(batteryVoltage, 2);
  }
  
  char radioPacket[60] = {'\0'}; // need a char array to send to the radio
  radioPacketString.toCharArray(radioPacket, 60);

  #ifdef SERIAL_DEBUG
  Serial.println(radioPacket); // troubleshooting
  #endif
  
  rf69.send((uint8_t *)radioPacket, strlen(radioPacket)); // send the packet
  rf69.waitPacketSent(); // wait until packet is sent before continuing
  rf69.sleep(); //put radio to sleep to save power
  digitalWrite(SHTPWRPIN, LOW); // turn off SHT45 for power saving while sleeping
  
  #ifdef SERIAL_DEBUG
  delay(8000);
  #else
  digitalWrite(LED, LOW); // turn off LED to indicate sleep
  Watchdog.sleep(16000); // sleep for 16 seconds (16 is maximum, but if you do between 8000 and 16000 will drop to 8)
  #endif
}

float wetBulbCalc(float DB, float RH){
  float WBC = DB*(atan(0.151977*pow(RH + 8.313659,0.5))) + atan(DB + RH) - atan(RH - 1.676331) + 0.00391838*pow(RH, 1.5)*atan(0.023101*RH) - 4.686035;
  return WBC;
}
