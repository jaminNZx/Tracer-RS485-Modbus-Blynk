// CONNECT THE RS485 MODULE RX->D5, TX->D6. (SoftwareSerial)
// You do not need to disconnect the RS485 while uploading code.
// Tested on NodeMCU + MAX485 module
// RJ 45 cable: Green -> A, Blue -> B
// MAX485: DE + RE interconnected with a jumper and connected to D3 or D4
// Developed by @jaminNZx
// With modifications by @tekk

#include <ArduinoOTA.h>
#include <BlynkSimpleEsp8266.h>
#include <SimpleTimer.h>
#include <ModbusMaster.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include "settings.h"

int timerTask1, timerTask2, timerTask3;
float battChargeCurrent = battDischargeCurrent = battOverallCurrent = battChargePower = 0.0f;
float bvoltage, ctemp, btemp, bremaining, lpower, lcurrent, pvvoltage, pvcurrent, pvpower;
float stats_today_pv_volt_min, stats_today_pv_volt_max;
uint8_t result;
bool rs485DataReceived = true;
bool loadPoweredOn = true;

#define MAX485_DE D3
#define MAX485_RE_NEG D4
#define BAUD_RATE 115200
#define BUFFER_SIZE 128

ModbusMaster node;
//SoftwareSerial swSer(D5, D6, false, BUFFER_SIZE);
SimpleTimer timer;

void preTransmission() {
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
  delay(1); // wait to MAX485 flushes everything to us
}

void postTransmission() {
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  delay(1); // wait for SoftwareSerial data to arrive 2 buffer
}

// a list of the regisities to query in order
typedef void (*RegistryList[])();

RegistryList Registries = {
  AddressRegistry_3100,
  AddressRegistry_311A,
  AddressRegistry_3300,
};

// keep log of where we are
uint8_t currentRegistryNumber = 0;

// function to switch to next registry
void nextRegistryNumber() {
  // better not use modulo, because after overlow it will start reading in incorrect order
  currentRegistryNumber = currentRegistryNumber + 1;
  if (currentRegistryNumber > ARRAY_SIZE(Registries)) {
    currentRegistryNumber = 0;
  }
}

// ****************************************************************************

void setup()
{
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  Serial.begin(BAUD_RATE);  
  Serial2.begin(BAUD_RATE);
  
  // Modbus slave ID 1
  node.begin(1, Serial2);
  
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  
  if (debug == 1)
    Serial.println("Connecting to Wifi...");
  
  WiFi.mode(WIFI_STA);
#if defined(USE_LOCAL_SERVER)
  Blynk.begin(AUTH, WIFI_SSID, WIFI_PASS, SERVER);
#else
  Blynk.begin(AUTH, WIFI_SSID, WIFI_PASS);
#endif

  if (debug == 1) {
    Serial.println("Connected.");
    Serial.println("Connecting to Blynk...");
  }

  while (Blynk.connect() == false) {}

  if (debug == 1) {
    Serial.println("Connected to Blynk.");
    Serial.println("Starting ArduinoOTA...");
  }
  
  ArduinoOTA.setHostname(OTA_HOSTNAME );
  ArduinoOTA.begin();

  if (debug == 1) {
    Serial.println("ArduinoOTA running.");
    Serial.println("Starting timed actions...");
  }
  
  timerTask1 = timer.setInterval(1000L, incrementRegistryNumber);
  timerTask2 = timer.setInterval(1000L, nextRegistryNumber);
  timerTask3 = timer.setInterval(1000L, updateBlynk);

  if (debug == 1) {
    Serial.println("Setup OK!");
    Serial.println("----------------------------");
    Serial.println();
  }
}

// --------------------------------------------------------------------------------

uint8_t setOutputLoadPower(bool state) {
  if (debug == 1) {
    Serial.print("Writing coil 0x0006 value to: ");
    Serial.println(state);
  }
  
  // Set coil at address 0x0006 (Force the load on/off)
  ESP.wdtDisable();
  return node.writeSingleCoil(0x0006, (uint8_t)state);
  ESP.wdtEnable(1);

  delay(1); // wait 4 buffer
}

// upload values
void updateBlynk() {
  Blynk.virtualWrite(vPIN_PV_POWER,                   pvpower);
  Blynk.virtualWrite(vPIN_PV_CURRENT,                 pvcurrent);
  Blynk.virtualWrite(vPIN_PV_VOLTAGE,                 pvvoltage);
  Blynk.virtualWrite(vPIN_LOAD_CURRENT,               lcurrent);
  Blynk.virtualWrite(vPIN_LOAD_POWER,                 lpower);
  Blynk.virtualWrite(vPIN_BATT_TEMP,                  btemp);
  Blynk.virtualWrite(vPIN_BATT_VOLTAGE,               bvoltage);
  Blynk.virtualWrite(vPIN_BATT_REMAIN,                bremaining);
  Blynk.virtualWrite(vPIN_CONTROLLER_TEMP,            ctemp);
  Blynk.virtualWrite(vPIN_BATTERY_CHARGE_CURRENT,     battChargeCurrent);
  Blynk.virtualWrite(vPIN_BATTERY_CHARGE_POWER,       battChargePower);
  Blynk.virtualWrite(vPIN_BATTERY_OVERALL_CURRENT,    battOverallCurrent);
  Blynk.virtualWrite(vPIN_LOAD_ENABLED,               loadPoweredOn);
}

// callback to on/off button state changes from the Blynk app
BLYNK_WRITE(vPIN_LOAD_ENABLED) {
  bool newState = (bool)param.asInt();
  
  if (debug == 1) {
    Serial.print("Setting load state output coil to value: ");
    Serial.println(newState);
  }
  
  setOutputLoadPower(newState);
  delay(1);
  
  if (debug == 1) {
    readOutputLoadState();
    Serial.print("Read Output Load state value: ");
    Serial.println(loadPoweredOn);
  }
}

bool readOutputLoadState() {
  ESP.wdtDisable();
  result = node.readHoldingRegisters(0x903D, 1);
  ESP.wdtEnable(1);

  delay(1);
  
  if (result == node.ku8MBSuccess) {
    loadPoweredOn = node.getResponseBuffer(0x00) & 0x02 > 0;

    if (debug == 1) {
      Serial.print("Set success. Load: ");
      Serial.println(loadPoweredOn);
    }
  } else {
    // update of status failed
    if (debug == 1)
      Serial.println("readHoldingRegisters(0x903D, 1) failed!");
  }
  return loadPoweredOn;
}

// reads Load Enable Override coil - not used
uint8_t checkLoadState() {
  if (debug == 1)
    Serial.print("Reading coil 0x0006... ");

  ESP.wdtDisable();
  result = node.readCoils(0x0006, 1);
  ESP.wdtEnable(1);
  delay(1);
  
  if (debug == 1) 
    Serial.println(result == node.ku8MBSuccess ? "success." : "failed.");
  
  if (result == node.ku8MBSuccess) {
    loadPoweredOn = (bool)node.getResponseBuffer(0);
  } else {
    if (debug == 1) {
      Serial.println("Failed to read coil 0x0006!");
    }
  }

  if (debug == 1) {
    Serial.print(" Value: ");
    Serial.println(loadPoweredOn);
  }
  
  return result;
}

void executeRegistryRead() {
  // exec a function of registry read (cycles between 3 different addresses)
  Registries[currentRegistryNumber]();
}

void AddressRegistry_3100() {
  ESP.wdtDisable();
  result = node.readInputRegisters(0x3100, 16);
  ESP.wdtEnable(1);

  delay(1); // wait for the data to arrive
  
  if (result == node.ku8MBSuccess)
  {
    pvvoltage = node.getResponseBuffer(0x00) / 100.0f;
    if (debug == 1) {
      Serial.print("PV Voltage: ");
      Serial.println(pvvoltage);
    }

    pvcurrent = node.getResponseBuffer(0x01) / 100.0f;
    if (debug == 1) {
      Serial.print("PV Current: ");
      Serial.println(pvcurrent);
    }

    pvpower = (node.getResponseBuffer(0x02) | node.getResponseBuffer(0x03) << 16) / 100.0f;
    if (debug == 1) {
      Serial.print("PV Power: ");
      Serial.println(pvpower);
    }
    
    bvoltage = node.getResponseBuffer(0x04) / 100.0f;
    if (debug == 1) {
      Serial.print("Battery Voltage: ");
      Serial.println(bvoltage);
    }
    
    battChargeCurrent = node.getResponseBuffer(0x05) / 100.0f;
    if (debug == 1) {
      Serial.print("Battery Charge Current: ");
      Serial.println(battChargeCurrent);
    }
    
    battChargePower = (node.getResponseBuffer(0x06) | node.getResponseBuffer(0x07) << 16)  / 100.0f;
    if (debug == 1) {
      Serial.print("Battery Charge Power: ");
      Serial.println(battChargePower);
    }

    lcurrent = node.getResponseBuffer(0x0D) / 100.0f;
    if (debug == 1) {
      Serial.print("Load Current: ");
      Serial.println(lcurrent);
    }

    lpower = (node.getResponseBuffer(0x0E) | node.getResponseBuffer(0x0F) << 16) / 100.0f;
    if (debug == 1) {
      Serial.print("Load Power: ");
      Serial.println(lpower);
    }

    
    
  } else {
    rs485DataReceived = false;
    if (debug == 1) {
      Serial.println("Read register 0x3100 failed!");
    }
  }
}


void AddressRegistry_311A() {
    ESP.wdtDisable();
    result = node.readInputRegisters(0x311A, 2);
    ESP.wdtEnable(1);
  
    delay(1); // wait for the data to arrive
    
    if (result == node.ku8MBSuccess)
    {    
      bremaining = node.getResponseBuffer(0x00) / 1.0f;
      if (debug == 1) {
        Serial.print("Battery Remaining %: ");
        Serial.println(bremaining);
      }
      
      btemp = node.getResponseBuffer(0x01) / 100.0f;
      if (debug == 1) {
        Serial.print("Battery Temperature: ");
        Serial.println(btemp);
      }
    } else {
      rs485DataReceived = false;
      if (debug == 1) {
        Serial.println("Read register 0x3100 failed!");
      }
    }
  }
}

void readStatisticalDataRegister() {
  ESP.wdtDisable();
  result = node.readInputRegisters(0x3300, 16);
  ESP.wdtEnable(1);
  
  if (result == node.ku8MBSuccess)
  {
    stats_today_pv_volt_max = node.getResponseBuffer(0x00) / 100.0f;
    if (debug == 1) {
      Serial.print("Stats Today PV Voltage MAX: ");
      Serial.println(stats_today_pv_volt_max);
    }
    
    stats_today_pv_volt_min = node.getResponseBuffer(0x01) / 100.0f;
    if (debug == 1) {
      Serial.print("Stats Today PV Voltage MIN: ");
      Serial.println(stats_today_pv_volt_min);
    }

    battOverallCurrent = ((int32_t)node.getResponseBuffer(0x1B) + node.getResponseBuffer(0x1C) << 16) / 100.0f;
    if (debug == 1) {
      Serial.print("Battery Discharge Current: ");
      Serial.println(battDischargeCurrent);
    } 
  } else {
    rs485DataReceived = false;
  }
}

void readStatisticalDataRegister() {
  ESP.wdtDisable();
  result = node.readInputRegisters(0x3300, 16);
  ESP.wdtEnable(1);
  
  if (result == node.ku8MBSuccess)
  {
    battOverallCurrent = ((int32_t)node.getResponseBuffer(0x1B) + node.getResponseBuffer(0x1C) << 16) / 100.0f;
    if (debug == 1) {
      Serial.print("Battery Discharge Current: ");
      Serial.println(battDischargeCurrent);
    } 
  } else {
    rs485DataReceived = false;
  }
}

void loop()
{
  Blynk.run();
  ArduinoOTA.handle();
  timer.run();
}



// --------------------------------------------------------------------------------
// EXAMPLE OF WRITING TO THE CONTROLLER - WILL ADD LATER

/*

  void tracerGet() {



  if (debug == 1) {
   //Serial.print("Beginning Loop ");
   //Serial.println(numLoops);
  }

  //Get Date and Time, and update Controller Data and Time

   if (numLoops == 10000) { // Get date and time every 10000 loops

     dtString = getTime();
     if (debug == 1) {
       //Serial.println(dtString);
     }
     dateDay = atoi(dtString.substring(5, 7).c_str());
     dateYear = atoi(dtString.substring(14, 16).c_str());
     timeHour = atoi(dtString.substring(17, 19).c_str());
     timeMinute = atoi(dtString.substring(20, 22).c_str());
     timeSecond = atoi(dtString.substring(23, 25).c_str());
     if (dtString.substring(8, 11) == "Jan") {
       dateMonth = 1;
     }
     if (dtString.substring(8, 11) == "Feb") {
       dateMonth = 2;
     }
     if (dtString.substring(8, 11) == "Mar") {
       dateMonth = 3;
     }
     if (dtString.substring(8, 11) == "Apr") {
       dateMonth = 4;
     }
     if (dtString.substring(8, 11) == "May") {
       dateMonth = 5;
     }
     if (dtString.substring(8, 11) == "Jun") {
       dateMonth = 6;
     }
     if (dtString.substring(8, 11) == "Jul") {
       dateMonth = 7;
     }
     if (dtString.substring(8, 11) == "Aug") {
       dateMonth = 8;
     }
     if (dtString.substring(8, 11) == "Sep") {
       dateMonth = 9;
     }
     if (dtString.substring(8, 11) == "Oct") {
       dateMonth = 10;
     }
     if (dtString.substring(8, 11) == "Nov") {
       dateMonth = 11;
     }
     if (dtString.substring(8, 11) == "Dec") {
       dateMonth = 12;
     }

     if (debug == 1) {
       Serial.print("Date: ");
       Serial.print(dateDay);
       Serial.print("/");
       Serial.print(dateMonth);
       Serial.print("/");
       Serial.print(dateYear);
       Serial.print("     ");
       Serial.print(timeHour);
       Serial.print(":");
       Serial.print(timeMinute);
       Serial.print(":");
       Serial.println(timeSecond);
       Serial.println(" ");
       Serial.println(timeSecond << 8 | timeMinute);
       Serial.println(lowByte(timeSecond << 8 | timeMinute));

       delay(2000);
     }
     node.setTransmitBuffer(0, (timeMinute << 8) | timeSecond);
     node.setTransmitBuffer(1, (dateDay << 8) | timeHour);
     node.setTransmitBuffer(2, (dateYear << 8) | dateMonth);
     result = node.writeMultipleRegisters(0x9013, 3);

     numLoops = 0;

   }   //End of get date and time

*/
/*
  if (debug == 1) {
    result = node.readHoldingRegisters(0x9013, 3);
    if (result == node.ku8MBSuccess)
    {
      if (debug == 1) {
        Serial.println();
        Serial.print("Time is: ");
      }
      time1 = lowByte(node.getResponseBuffer(0x00));
      time2 = highByte(node.getResponseBuffer(0x00));
      time3 = lowByte(node.getResponseBuffer(0x01));

      sprintf(buf, "%02d:%02d:%02d", time3, time2, time1);
      Serial.println(buf);

      Serial.println(time2 << 8 | time1);
      Serial.println(node.getResponseBuffer(0x00));

      Serial.println((time2 << 8 | time1) == (node.getResponseBuffer(0x00)));
      date1 = (node.getResponseBuffer(0x01) >> 8);
      Serial.println(time3);
      Serial.print("Date is: ");
      Serial.println(date1);;
      date2 = node.getResponseBuffer(0x02) & 0xff;
      date3 = (node.getResponseBuffer(0x02) >> 8);
      Serial.println(date2);
      Serial.println(date3);
      Serial.println();
    } else {
      rs485DataReceived = false;
    }

  }

  }

*/

