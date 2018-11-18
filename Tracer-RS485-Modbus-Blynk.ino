// CONNECT THE RS485 MODULE RX->D7, TX->D8. (SerialModbus is on UART2)
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

#ifndef Serial2
#include <HardwareSerial.h>
#endif

#include "settings.h"

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

int timerTask1, timerTask2, timerTask3, timerTask4;
float battChargeCurrent, battDischargeCurrent, battOverallCurrent, battChargePower;
float bvoltage, ctemp, btemp, bremaining, lpower, lcurrent, pvvoltage, pvcurrent, pvpower;
float stats_today_pv_volt_min, stats_today_pv_volt_max;
uint8_t result;
bool rs485DataReceived = true;
bool loadPoweredOn = true;

#define MAX485_DE D3
#define MAX485_RE_NEG D4
#define BAUD_RATE 115200

#ifdef Serial2
  #define SerialModbus Serial2
#else
  HardwareSerial SerialModbus(2);
#endif

ModbusMaster node;
SimpleTimer timer;

void preTransmission() {
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission() {
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

// a list of the regisities to query in order
typedef void (*RegistryList[]) ();

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
  if (currentRegistryNumber > ARRAY_SIZE(Registries) ) {
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
  SerialModbus.begin(BAUD_RATE, SERIAL_8N1, SERIAL_FULL, D8);
  
  // Modbus slave ID 1
  node.begin(1, SerialModbus);

  // function to be called in the idle time between transmission of data and response from slave
  node.idle(flush_buffers);

  // callbacks to toggle DE + RE on MAX485
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
  
  timerTask1 = timer.setInterval(1000L, executeCurrentRegistryFunction);
  timerTask2 = timer.setInterval(1000L, nextRegistryNumber);
  timerTask3 = timer.setInterval(1000L, checkLoadCoilState);
  timerTask4 = timer.setInterval(1000L, uploadToBlynk);

  if (debug == 1) {
    Serial.println("Setup OK!");
    Serial.println("----------------------------");
    Serial.println();
  }
}

// --------------------------------------------------------------------------------

// upload values
void uploadToBlynk() {
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

// exec a function of registry read (cycles between different addresses)
void executeCurrentRegistryFunction() {
  Registries[currentRegistryNumber]();
}


uint8_t setOutputLoadPower(bool state) {
  if (debug == 1) {
    Serial.print("Writing coil 0x0006 value to: ");
    Serial.println(state);
  }
  
  // Set coil at address 0x0006 (Force the load on/off)
  ESP.wdtDisable();
  return node.writeSingleCoil(0x0006, (uint8_t)state);
  ESP.wdtEnable(1);

  flush_buffers(); // wait 4 data to arrive
}

// callback to on/off button state changes from the Blynk app
BLYNK_WRITE(vPIN_LOAD_ENABLED) {
  bool newState = (bool)param.asInt();
  
  if (debug == 1) {
    Serial.print("Setting load state output coil to value: ");
    Serial.println(newState);
  }
  
  setOutputLoadPower(newState);
  flush_buffers();
  
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

  flush_buffers();
  
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

// reads Load Enable Override coil
void checkLoadCoilState() {
  if (debug == 1) {
    Serial.print("Reading coil 0x0006... ");
  }
  
  ESP.wdtDisable();
  result = node.readCoils(0x0006, 1);
  ESP.wdtEnable(1);
  flush_buffers();
  
  if (debug == 1) {
    Serial.println(result == node.ku8MBSuccess ? "success." : "failed.");
  }
  
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
}

// -----------------------------------------------------------------

  void AddressRegistry_3100() {
    ESP.wdtDisable();
    result = node.readInputRegisters(0x3100, 16);
    ESP.wdtEnable(1);
  
    flush_buffers(); // wait for the data to arrive
    
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
    flush_buffers();
   
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
        Serial.println("Read register 0x311A failed!");
      }
    }
  }

  void AddressRegistry_3300() {
    ESP.wdtDisable();
    result = node.readInputRegisters(0x3300, 16);
    ESP.wdtEnable(1);
    flush_buffers();
    
    if (result == node.ku8MBSuccess) {
      battOverallCurrent = (node.getResponseBuffer(0x1B) | node.getResponseBuffer(0x1C) << 16) / 100.0f;
      if (debug == 1) {
        Serial.print("Battery Discharge Current: ");
        Serial.println(battOverallCurrent);
      } 
    } else {
      rs485DataReceived = false;
      if (debug == 1) {
        Serial.println("Read register 0x3300 failed!");
      }    
    }
  }

void flush_buffers() {
  const byte delMicro = 3;

  static uint64_t startTime = millis();
  while (SerialModbus.available() && (startTime + delMicro > millis())) {
    yield();
  }

  startTime = millis();
  while (Serial.available() && (startTime + delMicro > millis())) {
    yield();
  }

  SerialModbus.flush();
  Serial.flush();
}

void loop()
{
  Blynk.run();
  ArduinoOTA.handle();
  timer.run();
  flush_buffers();
}

