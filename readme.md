#  VMC unelvent IDEO 325 radio gateway

## Introduction

This project is an VMC unelvent IDEO 325 radio gateway.
The idea is to have an raspberry pi with a cc1101 radio gateway that will act as an IDEO 325 / MQQT gateway.
This will enable to exploit the IDEO 325 VMC from a remote home automation platform like Openhab.

The gateway is bidirectionnal.

From the VMC, you will be able to retrieve information like:

- Temperature : external, input, stale air.
- VMC fan mode : low, high, away.
- Bypass mode : off, auto, forced.

You will also be able to send command to the VMC:
- force Bypass mode.
- Switch VMC on/off.
- Set low and high fan speed.
- Set Holiday mode.
- Set schedule mode : Auto1, Auto2, Manual.
- Toggle boost mode.

Today only bypass cmd are implemented, but other command will come soon !

## Special thanks

A big thanks to Guillaume Strub that make the original project on github https://github.com/lattic/rf2mqtt/ with an arduino and cc1101.
He show me the way to go with it's code. It's him that first work on the reverse enginnering of the radio protocol.

## Electronics

17	3,3v
19	MOSI
21	MISO
23	SCLK
25	GND

22
24
26


241/0

CC1101	Raspberry Pi
VDD		3.3V (Pin 1 or 17)
SI		MOSI (Pin 19)
SO		MISO (Pin 21)
CSn		  (Pin 24)
SCLK	SCLK (Pin 23)
GDO2*	Any GPIO pin, commonly GPIO25 (Pin 22) [1,2,3]
GDO0*	Any GPIO pin, GPIO24 (Pin 18) recommended
GND		Ground

0x00/0x14	0/20

https://github.com/f4exb/picc1101
 868.335200
 
 
 MOSI 		OUT		12	19		ALT0
 MISO		IN		13	21		ALT0
 SCLK		OUT		14	23		ALT0
 CE0		OUT		24	10		OUT
 CE1		OUT		26	11		OUT
 GPIO.6 	IN		18	5		-
 GPIO.5 	IN		22	6		-

 ## Reference
 
 https://docs.openmqttgateway.com/use/rf.html#what-is-an-rf-gateway
 
 
 
https://github.com/wladimir-computin/CC1101-ESP-Arduino
https://github.com/nopnop2002/esp-idf-cc1101
https://forum.hacf.fr/t/espsomfy-rts-une-autre-solution-pour-la-gestion-de-vos-volets-somfy-rts/27120/6?page=2
https://github.com/rstrouse/ESPSomfy-RTS/wiki/Installing-the-Firmware#using-esphome-web
https://www.amazon.fr/gp/aw/d/B071P98VTG?tag=hacf0d-21&th=1
https://github.com/rstrouse/ESPSomfy-RTS-HA

https://github.com/lattic/rf2mqtt/blob/main/firmware/IdeoManager.cpp

https://github.com/rstrouse/ESPSomfy-RTS/wiki/Installing-the-Firmware#using-esphome-web
https://haade.fr/fr/blog/domotiser-compteur-eau-itron-everblu-cyble

https://www.reddit.com/r/esp32/comments/1d7wogu/iboost_monitor_esp32_cc1101_display/?tl=fr
https://www.rtl-sdr.com/rtl_433-ported-to-esp32-microcontrollers-with-cc1101-or-sx127x-transceiver-chips/
OpenMqttGateway 
OpenMqttGateway 
Tasmota firmware 
https://docs.openmqttgateway.com/
https://tasmota.github.io/docs/
https://esp-32.com/

https://www.securipi.co.uk/cc1101.pdf
https://www.raspberrypi.com/documentation/computers/raspberry-pi.html
https://www.ti.com/lit/ds/symlink/cc1101.pdf
https://github.com/fphammerle/python-cc1101/blob/master/cc1101/__init__.py
https://github.com/SpaceTeddy/CC1101
https://github.com/puuu/ESPiLight
https://github.com/LSatan/SmartRC-CC1101-Driver-Lib



1111 1111 1111 1111

