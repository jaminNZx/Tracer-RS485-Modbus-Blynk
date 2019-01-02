/**************************************************************
 *
 *                  Settings - Tracer 
 *
 **************************************************************/
/*
   Auth Codes & Wifi info go in the following file.
   Create a new folder in your library dir called 'esp_credentials'
   and create a new file called 'esp_credentials.h' or move the 
   example dir in the repo. You can use this for all your wifi projects. 
   
   Example esp_credentials.h is in ./esp-credentials/esp-credentials.h
*/

// include WIFI credentials and Blynk auth token credentials
#include "esp_credentials.h"

/*
   Local Server Settings
   Comment out to use Cloud Server
*/
//#define USE_LOCAL_SERVER
//#define SERVER                          IPAddress(192, 168, 1, 78)

/*
  Over The Air Hostname  
*/
#define OTA_HOSTNAME "SOLAR-MODBUS"

/*
   Virtual Pins - Base. (For Blynk)
*/
#define vPIN_PV_POWER                   V1
#define vPIN_PV_CURRENT                 V2
#define vPIN_PV_VOLTAGE                 V3
#define vPIN_LOAD_CURRENT               V4
#define vPIN_LOAD_POWER                 V5
#define vPIN_BATT_TEMP                  V6
#define vPIN_BATT_VOLTAGE               V7
#define vPIN_BATT_REMAIN                V8
#define vPIN_CONTROLLER_TEMP            V9
#define vPIN_BATTERY_CHARGE_CURRENT     V10
#define vPIN_BATTERY_CHARGE_POWER       V11
#define vPIN_BATTERY_OVERALL_CURRENT    V12
#define vPIN_LOAD_ENABLED               V14
