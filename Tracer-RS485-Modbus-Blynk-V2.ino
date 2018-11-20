// CONNECT THE RS485 MODULE. ESP8266 RX = D7, TX = D8. (SerialModbus is on UART2)
// You do not need to disconnect the RS485 while uploading code.
// Tested on NodeMCU + MAX485 module
// RJ 45 cable: Green -> A, Blue -> B, Brown -> GND module + GND ESP8266
// MAX485: DE + RE interconnected with a jumper and connected to D1 or D2
// Developed by @jaminNZx
// With modifications by @tekk

#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <BlynkSimpleEsp8266.h>
#include <SimpleTimer.h>
#include <ModbusMaster.h>
#include "settings.h"

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

const int defaultBaudRate = 115200;
int timerTask1, timerTask2, timerTask3, timerTask4;
float battChargeCurrent, battDischargeCurrent, battOverallCurrent, battChargePower;
float bvoltage, ctemp, btemp, bremaining, lpower, lcurrent, pvvoltage, pvcurrent, pvpower;
float stats_today_pv_volt_min, stats_today_pv_volt_max;
uint8_t result;
bool rs485DataReceived = true;
bool loadPoweredOn = true;

#define MAX485_DE D1
#define MAX485_RE_NEG D2

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

// A list of the regisities to query in order
typedef void (*RegistryList[])();

RegistryList Registries = {
  AddressRegistry_3100,
  AddressRegistry_3106,
  AddressRegistry_310D,
  AddressRegistry_311A,
  AddressRegistry_3300,
};

// keep log of where we are
uint8_t currentRegistryNumber = 0;

// function to switch to next registry
void nextRegistryNumber() {
  // better not use modulo, because after overlow it will start reading in incorrect order
  currentRegistryNumber++;
  if (currentRegistryNumber >= ARRAY_SIZE(Registries) ) {
    currentRegistryNumber = 0;
  }
}

// ****************************************************************************

void setup()
{
  // overcurrent protection
  delay(1000);

  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  
//  Serial.begin(defaultBaudRate, SERIAL_8N1, SERIAL_FULL, 1);
//  SerialModbus.begin(defaultBaudRate, SERIAL_8N1, SERIAL_FULL, 15);

  Serial.begin(defaultBaudRate);

  // Modbus slave ID 1
  node.begin(1, Serial);

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

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  ArduinoOTA.setHostname(OTA_HOSTNAME );

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  
  ArduinoOTA.begin();

  if (debug == 1) {
    Serial.print("ArduinoOTA running. ");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("Starting timed actions...");
  }
  
  timerTask1 = timer.setInterval(1000L, executeCurrentRegistryFunction);
  timerTask2 = timer.setInterval(1000L, nextRegistryNumber);
  timerTask3 = timer.setInterval(1000L, uploadToBlynk);

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
    return node.writeSingleCoil(0x0006, (uint8_t)state);
  }
  
  // callback to on/off button state changes from the Blynk app
  BLYNK_WRITE(vPIN_LOAD_ENABLED) {
    bool newState = (bool)param.asInt();
    
    if (debug == 1) {
      Serial.print("Setting load state output coil to value: ");
      Serial.println(newState);
    }
    
    setOutputLoadPower(newState);
    
    if (debug == 1) {
      readOutputLoadState();
      Serial.print("Read Output Load state value: ");
      Serial.println(loadPoweredOn);
    }
  }
  
  bool readOutputLoadState() {
    result = node.readHoldingRegisters(0x903D, 1);
    
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
    
    result = node.readCoils(0x0006, 1);
    
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
    result = node.readInputRegisters(0x3100, 6);
  
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
    }
  }

  void AddressRegistry_3106()
  {
    result = node.readInputRegisters(0x3106, 2);

    if (result == node.ku8MBSuccess)
    {
      battChargePower = (node.getResponseBuffer(0x00) | node.getResponseBuffer(0x01) << 16)  / 100.0f;
      if (debug == 1) {
        Serial.print("Battery Charge Power: ");
        Serial.println(battChargePower);
      }
    }
  }

  void AddressRegistry_310D()
  {
    result = node.readInputRegisters(0x310D, 3);

    if (result == node.ku8MBSuccess)
    {
      lcurrent = node.getResponseBuffer(0x00) / 100.0f;
      if (debug == 1) {
        Serial.print("Load Current: ");
        Serial.println(lcurrent);
      }
  
      lpower = (node.getResponseBuffer(0x01) | node.getResponseBuffer(0x02) << 16) / 100.0f;
      if (debug == 1) {
        Serial.print("Load Power: ");
        Serial.println(lpower);
      }
    } else {
      rs485DataReceived = false;
      if (debug == 1) {
        Serial.println("Read register 0x310D failed!");
      }
    }    
  }
 


  void AddressRegistry_311A() {
    result = node.readInputRegisters(0x311A, 2);
   
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
    result = node.readInputRegisters(0x331B, 2);
    
    if (result == node.ku8MBSuccess) {
      battOverallCurrent = (node.getResponseBuffer(0x00) | node.getResponseBuffer(0x01) << 16) / 100.0f;
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
  const byte delMicro = 10;

  static uint64_t startTime = millis();

  startTime = millis();
  while (Serial.available() && (startTime + delMicro >= millis())) {
    yield();
  }
}

void loop()
{
  Blynk.run();
  ArduinoOTA.handle();
  timer.run();
}
