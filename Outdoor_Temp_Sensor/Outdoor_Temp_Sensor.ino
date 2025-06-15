/*
Sketch for outdoor unit of indoor/outdoor "which is better" project to determine if opening windows or keeping them closed feels better.
Indoor unit has graphic LCD display & RTC with SD card logging & SHT45 RH/temp sensor.
Outdoor unit has RHT45 sensor and goes to sleep to save battery.
Both units are built on a M0 powered Feather with RFM69 915 MHz radio (USA ISM frequency).

2024-01-27: Added Unit ID to transmission for multiple remote units
 
Cut e-ink ECS pin, soldered wire to move it to pin A1 on feather (GPIO 15 on M0 Feather)
Cut SDCS pin on RTC FeatherWing, jumpered to pin A2 on feather (GPIO 16 on M0 Feather)


Copyright 2024, 2025 Jonathan Seyfert

This file is part of Indoor-Outdoor_Temp_Sensor.

Indoor-Outdoor_Temp_Sensor is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License
as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

Indoor-Outdoor_Temp_Sensor is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Indoor-Outdoor_Temp_Sensor. If not, see <https://www.gnu.org/licenses/>. 
*/

#include <SPI.h>  // For RFM69
#include <RH_RF69.h> // For RFM69
#include <RHReliableDatagram.h> // For RFM69
#include <Adafruit_SHT4x.h> // for SHT45 temp/humidity sensor
#include <Adafruit_SleepyDog.h> // for watchdog sleep for power savings
#include <json.hpp> // Must place this in your Arduino IDE libraries folder in a folder called "json"

#define RF69_FREQ 915.0  // Frequency of RFM69
#define RFM69_CS      8  // RFM69 SPI chip select pin
#define RFM69_INT     3  // RFM69 interrupt pin
#define RFM69_RST     4  // RFM69 reset pin
#define LED           13 // Built-in LED on M0 Feather
#define VBATPIN       A7 // Internal battery voltage divider measurement pin
#define SHTPWRPIN     5  // Pin to supply power to SHT45 to turn it on and off for power savings

#define SERIAL_DEBUG // comment out if serial debugging is not desired. Watchdog sleep will be disabled if defined

extern "C" char *sbrk(int i);  // for FreeMem()
float x = 0.0; // for custom dtostrf()
float y = 0.0; // for custom dtostrf()
float batteryVoltage = 0; // for measuring battery voltage
const uint8_t rxAddress = 1; // Address of base station
const uint8_t unitId = 2;    // Address of this unit. Change as desired
// Unit 1 is outdoor. Unit 2 is kegerator.
// SHOULD ADD: Some dip switches that allow setting unitID instead of needing it to be compiled into code (thought that would eat up a lot of IO)

Adafruit_SHT4x sht4 = Adafruit_SHT4x(); // for SHT45
RH_RF69 driver(RFM69_CS, RFM69_INT); // Singleton instance of the radio driver
RHReliableDatagram manager(driver, unitId); // Class to manage message delivery and receipt, using the driver declared above

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

  pinMode(LED, OUTPUT);

  pinMode(SHTPWRPIN, OUTPUT); // Initialize pin for SHT45 power control
  digitalWrite(SHTPWRPIN, HIGH); //turn on SHT45 for initialization
  delay(100);
  sht4.begin(); // initialize SHT45 sensor
  sht4.setPrecision(SHT4X_HIGH_PRECISION); // can use MED or LOW, HIGH takes longer
  sht4.setHeater(SHT4X_NO_HEATER); // can use 6 different heater options, see example
  
  manager.init(); // Initialize RFM69
  driver.setFrequency(RF69_FREQ); // Set RFM69 frequency
  driver.setTxPower(20, true);  // Power levels can be set from -2 to 20, 2nd arg must be true for RFM69. 1% duty cycle at 20, VSWR 3:1 max
  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  driver.setEncryptionKey(key);
}

void loop()
{
  //digitalWrite(LED, HIGH); // turn on LED to indicate not sleeping
  float dryBulb = 0.00;
  float wetBulb = 0.00;
  unsigned long tempTime = 0; // mostly for troubleshooting right now
  unsigned long rhTime = 0; // mostly for troubleshooting right now
  sensors_event_t humidity, temp; // for SHT45

  for(int i = 1; i <= 10; i++)
  {
    digitalWrite(SHTPWRPIN, HIGH); // turn on SHT45
    delay(100); // give it some time to powerup
    if(sht4.getEvent(&humidity, &temp)) // check if our request for temp and humidity worked
    {
      nlohmann::json message; // empty json struction to hold message we want to send
      message["dry_bulb"] = round(temp.temperature*10)/10.0; // add dry bulb temp to json, rounding to 1 decimal place
      message["humidity"] = round(humidity.relative_humidity*10)/10.0; // add RH to json, rounding to 1 decimal place
      message["wet_bulb"] = round(wetBulbCalc(temp.temperature, humidity.relative_humidity)*10)/10.0; // add web bulb temp to json, round to 1 decimal place
      message["battery_voltage"] = round(analogRead(VBATPIN)*0.006445*100)/100; // ADC*2*3.3/1024 *2 - 50% voltage divider. *3.3 - V_ref. /1024 - 8-bit ADC. Round 2 decimal places

      String jsonMsg = String(message.dump().c_str());

      #ifdef SERIAL_DEBUG
      Serial.print("json contents: ");
      Serial.println(jsonMsg);
      #endif

      uint8_t numPacs = jsonMsg.length()/57 + 1; // each packet sent can be 60 characters long, but need to leave one char for EOM, so we get 59 sendable chars per packet, - 2 for X of Y pac #
      char packets[numPacs][60] = {'\0'}; // array of char arrays to hold packets for sending
      for (uint8_t i = 0; i < numPacs; i++) // sub-device json String into 59 char log strings, then convert sub-strings to char arrays and feed into array of char packets
      {
        String s = String(String(char(i + 1)) + String(char(numPacs))); // packet header: X packet of Y # packets. Typecast first and second chars of packet to int to decode on Rx.
        s += jsonMsg.substring(i*57, i*57 + 57); // concat 57 chars of message after X of Y header, leaving 1 char for Null termination of char string
        s.toCharArray(packets[i], 60); // Convert String to char string for sending via packet radio
      }

      #ifdef SERIAL_DEBUG
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
        if (!manager.sendtoWait((uint8_t *)packets[i], sizeof(packets[i]), rxAddress))
          allSent = true;
      }
      if (allSent == false)
      {
        #ifdef SERIAL_DEBUG
        Serial.println("Received an ack for sent messages.");
        #endif
        digitalWrite(LED, LOW);
      } 
      else
      {
        #ifdef SERIAL_DEBUG
        Serial.println("Did not receive ack for one or more of the sent messages!");
        #endif
        digitalWrite(LED, HIGH); // turn on LED to indicate failure to recieve ack
      }
      break; // leave the for loop
    }
    else
    {
      digitalWrite(SHTPWRPIN, LOW);
      delay(50);

      #ifdef SERIAL_DEBUG
      Serial.print("failed to get event #");
      Serial.println(i);
      #endif
    }
  }
  
  // dryBulb = temp.temperature;
  
  // wetBulb = wetBulbCalc(dryBulb, humidity.relative_humidity);
  // batteryVoltage = analogRead(VBATPIN); // read battery voltage
  // batteryVoltage *= 0.006445;    // ADC*2*3.3/1024 (Multiply ADC value by 2 because of 50% voltage divider, multiply by 3.3 for V_ref, divide by 1024 for 8-bit ADC)

  // String radioPacketString = "";
  // if(!eventGood) { 
  //   radioPacketString = "---- ---- ---- " + String(batteryVoltage, 2); // we don't have new data, so don't send new data
  // }
  // else {
  //   radioPacketString = String(dryBulb*1.8 + 32, 1) + " " + String((0.7*wetBulb + 0.3*dryBulb)*1.8 + 32, 1) + " "
  //                                      + String(humidity.relative_humidity, 1) + " " + String(batteryVoltage, 2);
  // }
  
  // char radioPacket[60] = {'\0'}; // need a char array to send to the radio
  // radioPacketString.toCharArray(radioPacket, 60);

  // #ifdef SERIAL_DEBUG
  // Serial.println(radioPacket); // troubleshooting
  // #endif
  
  // // rf69.send((uint8_t *)radioPacket, strlen(radioPacket)); // send the packet
  // // rf69.waitPacketSent(); // wait until packet is sent before continuing
  // // rf69.sleep(); //put radio to sleep to save power
  // if (manager.sendtoWait((uint8_t *)radioPacket, sizeof(radioPacket), rxAddress)) // Send the packet to the base station
  // {
  //   #ifdef SERIAL_DEBUG
  //   Serial.println("Received an ack for sent message.");
  //   #endif
  //   digitalWrite(LED, LOW);
  // } 
  // else
  // {
  //   #ifdef SERIAL_DEBUG
  //   Serial.println("Did not receive ack for sent message!");
  //   #endif
  //   digitalWrite(LED, HIGH); // turn on LED to indicate failure to recieve ack
  // }
  // in the future, try doing an if (manager.sendtoWait() and do something if it doesn't send. true is returned if it gets ack, false if not
  digitalWrite(SHTPWRPIN, LOW); // turn off SHT45 for power saving while sleeping
  
  #ifdef SERIAL_DEBUG
  delay(8000);
  #else
  //digitalWrite(LED, LOW); // turn off LED to indicate sleep
  Watchdog.sleep(16000); // sleep for 16 seconds (16 is maximum, but if you do between 8000 and 16000 will drop to 8)
  #endif
}

float wetBulbCalc(float DB, float RH){
  float WBC = DB*(atan(0.151977*pow(RH + 8.313659,0.5))) + atan(DB + RH) - atan(RH - 1.676331) + 0.00391838*pow(RH, 1.5)*atan(0.023101*RH) - 4.686035;
  return WBC;
}

int FreeMem () {  //http://forum.arduino.cc/index.php?topic=365830.msg2542879#msg2542879
  char stack_dummy = 0;
  return &stack_dummy - sbrk(0);
}
