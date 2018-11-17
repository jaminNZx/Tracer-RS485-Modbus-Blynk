/**************************************************************
 *
 *                  Settings - Tracer 
 *
 **************************************************************/
/*
   Auth Codes & Wifi info go in the following file.
   Create a new folder in your library dir called 'wifi_credentials'
   and create a new file called 'wifi_credentials.h' and copy the 
   example in the repo. You can use this for all your wifi projects. 
   
   Example wifi_credentials.h will look like this:
*/

#define WIFI_SSID             "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
#define WIFI_PASS             "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"

// Blynk API key
#define AUTH                  "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"

/*
   Local Server Settings
   Comment out to use Cloud Server
*/
//#define USE_LOCAL_SERVER       
//#define SERVER                          IPAddress(192, 168, 1, 78)

/*
  Over The Air Hostname  
*/
#define OTA_HOSTNAME "MPPT-MODBUS"

/*
   Virtual Pins - Base
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

/*
   Debug. Change to 0 when you are finished debugging.
*/
const int debug             =           1;

