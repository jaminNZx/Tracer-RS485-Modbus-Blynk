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
*/
#include <wifi_credentials.h>
/*
     Blynk Auth Codes
*/
#define AUTH                            "61482d630a864822949fc3fb1ad431a5"
/*
   Local Server Settings
   Comment out to use Cloud Server
*/
#define USE_LOCAL_SERVER       
#define SERVER                          IPAddress(192, 168, 1, 2)

/*
  Over The Air Hostname  
*/
#define OTA_HOSTNAME                    "SOLAR-CHARGE-MONITOR"
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
/*
   Debug. Change to 0 when you are finished debugging.
*/
const int debug             =           1; 
/*
   
*/
