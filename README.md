# Tracer-RS485-Modbus-Blynk
An arduino sketch to connect the EPSolar/EPEver Tracer A/B Series (RS-485 Modbus) to an ESP8266 and monitor using the Blynk mobile app.

Feel free to make pull requests if you wish to help develop it. 

There is also a support forum on the Blynk community forums: http://community.blynk.cc/t/epsolar-tracer-2210a-charge-controller-blynk-epic-solar-monitor/10596

## Hardware

* [EPSolar/EPEver Tracer A/B-Series](https://www.aliexpress.com/wholesale?catId=0&initiative_id=SB_20170114172728&SearchText=tracer+mppt+rs485)

* [RS485 UART Module](https://www.aliexpress.com/wholesale?catId=0&initiative_id=SB_20170114172807&SearchText=uart+rs485)

* [ESP8266 Dev Board](https://www.aliexpress.com/wholesale?catId=0&initiative_id=SB_20170114172938&SearchText=esp8266+mini)

* An old ethernet cable with RJ45 connector you are happy to cut open

## Software

* [Blynk](http://www.blynk.cc/) Mobile App ([iOS](https://itunes.apple.com/us/app/blynk-iot-for-arduino-rpi/id808760481?mt=8) & [Android](https://play.google.com/store/apps/details?id=cc.blynk&hl=en))
* Arduino IDE 1.6.9+
* The project sketch

## Wiring

Cut open your ethernet cable and split out pin 3,5,7 (B,A,GND)

Follow the wiring guide below:
![Tracer Wiring Diagram](http://i.imgur.com/OktbhPG.png)

## Setup

Open the Blynk mobile app and create a new project
Send yourself the generate auth code
Paste your auth code in to the sketch


