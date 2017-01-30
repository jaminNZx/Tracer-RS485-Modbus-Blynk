

// CONNECT THE RS485 MODULE RX->RX, TX->TX.
// Disconnect when uploading code.

#include <ArduinoOTA.h>
#include <BlynkSimpleEsp8266.h>
#include <SimpleTimer.h>
#include <ModbusMaster.h>
#include <ESP8266WiFi.h>
#include "settings.h"
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

int timerTask1, timerTask2, timerTask3;

float battBhargeCurrent, bvoltage, ctemp, btemp, bremaining, lpower, lcurrent, pvvoltage, pvcurrent, pvpower;
float stats_today_pv_volt_min, stats_today_pv_volt_max;
uint8_t result;

// To add later
//uint8_t result, time1, time2, time3, date1, date2, date3, dateDay, dateMonth, dateYear, timeHour, timeMinute, timeSecond;
//char buf[10];
//String dtString;

bool rs485DataReceived = true;

ModbusMaster node;
SimpleTimer timer;

void preTransmission() {}
void postTransmission() {}

typedef void (*RegistryList[])();
RegistryList Registries = {
  AddressRegistry_3100,
  AddressRegistry_311A,
  AddressRegistry_3300,
};

uint8_t currentRegistryNumber = 0;

void nextRegistryNumber() {
  currentRegistryNumber = (currentRegistryNumber + 1) % ARRAY_SIZE( Registries);
}

void setup()
{
  Serial.begin(115200);
  // Modbus slave ID 1
  node.begin(1, Serial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  WiFi.mode(WIFI_STA);
#if defined(USE_LOCAL_SERVER)
  Blynk.begin(AUTH, WIFI_SSID, WIFI_PASS, SERVER);
#else
  Blynk.begin(AUTH, WIFI_SSID, WIFI_PASS);
#endif
  while (Blynk.connect() == false) {}
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.begin();

  timerTask1 = timer.setInterval(1000, updateBlynk);
  timerTask2 = timer.setInterval(1000, doRegistryNumber);
  timerTask3 = timer.setInterval(1000, nextRegistryNumber);
}

// --------------------------------------------------------------------------------

void updateBlynk() {
  Blynk.virtualWrite(vPIN_PV_POWER,         pvpower);
  Blynk.virtualWrite(vPIN_PV_CURRENT,       pvcurrent);
  Blynk.virtualWrite(vPIN_PV_VOLTAGE,       pvvoltage);
  Blynk.virtualWrite(vPIN_LOAD_CURRENT,     lcurrent);
  Blynk.virtualWrite(vPIN_LOAD_POWER,       lpower);
  Blynk.virtualWrite(vPIN_BATT_TEMP,        btemp);
  Blynk.virtualWrite(vPIN_BATT_VOLTAGE,     bvoltage);
  Blynk.virtualWrite(vPIN_BATT_REMAIN,      bremaining);
  Blynk.virtualWrite(vPIN_CONTROLLER_TEMP,  ctemp);
}

void doRegistryNumber() {
  Registries[currentRegistryNumber]();
}

void AddressRegistry_3100() {
  result = node.readInputRegisters(0x3100, 7);
  if (result == node.ku8MBSuccess)
  {
    ctemp = node.getResponseBuffer(0x11) / 100.0f;
    if (debug == 1) {
      Serial.println(ctemp);
      Serial.print("Battery Voltage: ");
    }
    bvoltage = node.getResponseBuffer(0x04) / 100.0f;
    if (debug == 1) {
      Serial.println(bvoltage);

    }
    lpower = ((long)node.getResponseBuffer(0x0F) << 16 | node.getResponseBuffer(0x0E)) / 100.0f;
    if (debug == 1) {
      Serial.print("Load Power: ");
      Serial.println(lpower);

    }
    lcurrent = (long)node.getResponseBuffer(0x0D) / 100.0f;
    if (debug == 1) {
      Serial.print("Load Current: ");
      Serial.println(lcurrent);

    }
    pvvoltage = (long)node.getResponseBuffer(0x00) / 100.0f;
    if (debug == 1) {
      Serial.print("PV Voltage: ");
      Serial.println(pvvoltage);

    }
    pvcurrent = (long)node.getResponseBuffer(0x01) / 100.0f;
    if (debug == 1) {
      Serial.print("PV Current: ");
      Serial.println(pvcurrent);

    }
    pvpower = ((long)node.getResponseBuffer(0x03) << 16 | node.getResponseBuffer(0x02)) / 100.0f;
    if (debug == 1) {
      Serial.print("PV Power: ");
      Serial.println(pvpower);
    }
    battBhargeCurrent = (long)node.getResponseBuffer(0x05) / 100.0f;
    if (debug == 1) {
      Serial.print("Battery Charge Current: ");
      Serial.println(battBhargeCurrent);
      Serial.println();
    }
  } else {
    rs485DataReceived = false;
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
      Serial.println();
    }
  } else {
    rs485DataReceived = false;
  }
}

void AddressRegistry_3300() {
  result = node.readInputRegisters(0x3300, 2);
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

