# Tracer-RS485-Modbus-Blynk V2.0

> `@tekk:`
> This is partially rewritten branch of the original project.
> You are welcome for suggestions, bugreports, and of course any further improvements of this code.


## Info

An Arduino sketch to connect the EPSolar/EPEver Tracer A/B Series MPPT Solar controller (RS-485 Modbus) to an ESP8266 and monitor it using the Blynk mobile app.

Feel free to make pull requests if you wish to help develop it. 

There is also a support forum on the Blynk community forums: http://community.blynk.cc/t/epsolar-tracer-2210a-charge-controller-blynk-epic-solar-monitor/10596

## Hardware

* [EPSolar/EPEver Tracer A/B-Series](https://www.aliexpress.com/wholesale?catId=0&initiative_id=SB_20170114172728&SearchText=tracer+mppt+rs485)

* [RS485 UART Module](https://www.aliexpress.com/wholesale?catId=0&initiative_id=SB_20170114172807&SearchText=uart+rs485) (~~not the MAX485 chip!~~ - `@tekk:` I'm using [MAX485 cheapo module](doc/max485_module.jpg) and it works fine!)

* [ESP8266 Dev Board](https://www.aliexpress.com/wholesale?catId=0&initiative_id=SB_20170114172938&SearchText=esp8266+mini)

* An old ethernet cable with RJ45 connector you are happy to cut open

## Software

* [Blynk](http://www.blynk.cc/) Mobile App ([iOS](https://itunes.apple.com/us/app/blynk-iot-for-arduino-rpi/id808760481?mt=8) & [Android](https://play.google.com/store/apps/details?id=cc.blynk&hl=en))
* Arduino IDE 1.6.9+
* The project sketch

## Wiring

Cut open your ethernet cable and split out pin 3,5,7 (B,A,GND). Refer to [Tracer Modbus PDF](doc/1733_modbus_protocol.pdf) for additional info.

Follow the wiring guide below: (note that the 2-pol switch is only needed during flashing)
![Tracer Wiring Diagram](doc/schematic.png)

## Setup

### Libraries

* Blynk Library
* ArduinoOTA
* SimpleTimer 

### Tutorial

# Create wifi_credentials.h library 

Firstly, create a folder in your sketch **OR** libraries folder called `esp_credentials`. Then create a new file call `esp_credentials.h` inside it.

Edit the file and enter the following template.
Change the details for your own wifi network. 

You will be able to use this file by including it in any sketch by entering ```#include <esp_credentials.h>```. (This is already present in the `settings.h` file, no need to add it to this project.)

```cpp
/**************************************************************
 *           Settings - Wifi Credentials
 **************************************************************/
#define WIFI_SSID             "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
#define WIFI_PASS             "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"

```

* Open the Blynk mobile app and create a new project by scanning the following QR code

![Project QR Code](http://i.imgur.com/xBEmJyJ.jpg)

* Send yourself the generated auth code
* Paste your auth code in to the `esp_credentials.h` file

```cpp
#define AUTH                  "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
```

* ~~Disconnect the TX/RX cables (or open the switch if you have one)~~
	* [@tekk](https://github.com/tekk): You don't have to do this anymore! This version is using Serial1 on pins `D13` and `D2` to communicate with RS485 module
* Upload the sketch to your ESP8266
* ~~Once uploaded, reconnect the TX/RX cables and plug the cable in to the Tracer COM port~~
	* [@tekk](https://github.com/tekk): Just plug the cable, but it is always a good idea to repower the MAX485 module between sketch uploads / Serial2 reconnects
* Load the Blynk project and hit the PLAY button to start receiving data

## Reference

[Tracer A/B Series MPPT Solar Controller - Modbus Protocol](doc/1733_modbus_protocol.pdf)

## MAX485 module
`@tekk:`
	
![This one worked for me](doc/max485_module.jpg)

I'm using this cheapo module and it works quite fine.
It's powered from +5V on ESP8266, and wired as following:

- MAX485 module <-> ESP8266:
	- `DI` -> `D7`
	- `RO` -> `D8`
	- `DE` and `RE` interconnected with a jumper and then connected do eighter `D3` or `D4`
	- `VCC` to `+5V` on ESP8266


- Tracer A/B MPPT - Ethernet cable to MAX485
	- Ethernet green, pin `5` -> `A`
	- Ethernet blue, pin `3` -> `B`
	- Ethernet brown, pin `7` -> `GND` on module **and** ESP8266 `GND` pin
	
	
## Developing further

> I plan to add more features and pull more data from the controller once I have my own solar system running.
> If you'd like to pick this up and have a go at adding features, I'll be happy to accept pull requests.

## `@tekk`'s Changelog
- TODO

## Credits

- `@jaminNZx:`
	- Thanks to subtafuge on [Reddit](https://www.reddit.com/r/esp8266/comments/59dt00/using_esp8266_to_connect_rs485_modbus_protocol/) for lending me his working Tracer RS485 code! 

- `@tekk:`
	- Feel free to [contact me](mailto:tekk.sk@gmail.com) about my code changes
	- Thanks to [@jaminNZx](https://github.com/jaminNZx) for the original code. Big up!