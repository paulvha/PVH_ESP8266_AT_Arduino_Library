PVH ESP8266 AT Arduino Library Version 1.0 / January 2018
===============================================================
This is an enhanced version of the original SparkFun ESP8266 AT Arduino Library

A large number of bugfixes have been applied + new features that the
ESP8266 supports, but are not implemented in the original libary

==============================================================

## VERSION 2.0 / PAULVHA / November 2018  / ESP8266-PVH-driver
   new options + bug fixes :
  - now support Hardwareserial, Hardwareserial1,  2 or 3 (NOTE-1)
  - improved searchbuffer to get better result
  - baudrate handling different to default 9600 now working
  - change reset handling routine
  - added hard-reset and enableResetpin option
  - changed names Pinmode, digitalRead and digitalWrite to Esppinmode,
    EspdigitalRead and EspdigitalWrite. This will prevent confusion
  - extended timeout different times
  - further optimized to lower bytes footprint
  - updated GPIO, loopback, talkback and other examples

NOTE-1:
For HardwareSerial connect
    set the on-board switch to HW UART (IMPORTANT*1)

    RXline from the UART connection is connected to RX1 pin ( *2)
    TXline from the UART connection is connected to TX1 pin ( *2)

*1 : As the switch is on HW, the TX and RX signals are not only connected to the UART, but
     also to pin 1 and 2 of the shield. These are already used by the Arduino for the USB connection.
     You MUST make sure that these pin 1 and 2 are NOT CONNECTED to the Arduino (just cut them off)

     Select the hardware interface with calling ESP8266 :

     esp8266.begin(WIFI_SERIAL_SPEED, WIFI_SERIAL_TYPE);

     Valid WIFI_SERIAL_TYPE are  :
        ESP8266_SOFTWARE_SERIAL (default)
        ESP8266_HARDWARE_SERIAL
        ESP8266_HARDWARE_SERIAL1
        ESP8266_HARDWARE_SERIAL2
        ESP8266_HARDWARE_SERIAL3

*2 : Pin is depending on the hardware UART and the board
=====================================================================

##  Examples

The original examples are available and have been little adjusted to work
on this library, without any further enhancements

The following new examples have been added for loopback, talkback and
controlling GPIO from a remote client.

    pvh_ESP8266_Shield_softAP_loopback
    pvh_ESP8266_Shield_softAP_gpio
    pvh_ESP8266_Shield_server_talkback
    pvh_ESP8266_Shield_server_loopback
    pvh_ESP8266_Shield_Server_gpio
    pvh_ESP8266_Shield_test_sub (testing additional library calls)

Connect as a client to a remote server
    pvh_ESP8266_Shield_Client

##  Workbooks

In the Extra directory you will find a lot of extra information.

The following workbooks.odt are available:

1. workbook_client_loopback
    connect as a client to a remote server. A test-server program for
    Linux ( Loop.c) is available in extra-folder.

2. workbook_softAP_loopback
    Create an Access Point server and perform loopback. A test-client
    program for Linux (tclient.c) is available in extra-folder.

3. workbook_softAP_loopback
    Create an Access Point server or server on the local network and
    perform different GPIO function on the Arduino Pins and/or the
    ESP8266 pins. A test-client program for Linux ((tclient.c) is available in extra-folder.

4. workbook_server_loopback or talkback
    Create a server on the local network and perform talkback (basic Chat) or
    loopback. A test-client program for Linux (tclient.c) is available in extra-folder.

##  calls and subroutines

The following coding documentation :

1. PVH_ESP8266 library commands.odt
    An overview of all the library calls, highlevel describtion and return.

2. available sketch routines.odt:
    An overview of the available Sketch routines and in which examples sketches
    they are available.

Below the original README.md from SparkFun

Paul van Haastrecht
November 2018

***************************************************************************************

SparkFun ESP8266 AT Arduino Library
========================================

![SparkFun ESP8266 WiFi Shield](https://cdn.sparkfun.com//assets/parts/1/0/5/3/8/13287-01.jpg)

[*SparkFun ESP8266 WiFi Shield (WRL-13287)*](https://www.sparkfun.com/products/13287)

An Arduino library for the SparkFun ESP8266 WiFi Shield. This library makes it easy to connect to WiFi networks, and also implements TCP/IP client and server.

Repository Contents
-------------------

* **/examples** - Example sketches for the library (.ino). Run these from the Arduino IDE.
* **/extras** - Additional documentation for the user. These files are ignored by the IDE.
* **/src** - Source files for the library (.cpp, .h).
* **keywords.txt** - Keywords from this library that will be highlighted in the Arduino IDE.
* **library.properties** - General library properties for the Arduino package manager.

Documentation
--------------

* **[Installing an Arduino Library Guide](https://learn.sparkfun.com/tutorials/installing-an-arduino-library)** - Basic information on how to install an Arduino library.
* **[Product Repository](https://github.com/sparkfun/ESP8266_WiFi_Shield)** - Main repository (including hardware files) for the ESP8266 WiFi Shield.
* **[Hookup Guide](https://learn.sparkfun.com/tutorials/esp8266-wifi-shield-hookup-guide)** - Basic hookup guide for the ESP8266 WiFi Shield.

Products that use this Library
---------------------------------

* [SparkFun ESP8266 WiFi Shield (WRL-13287)](https://www.sparkfun.com/products/13287) - An Arduino shield featuring the ESP8266 WiFi SoC.

Version History
---------------

License Information
-------------------

This product is _**open source**_!

The **code** is beerware; if you see me (or any other SparkFun employee) at the local, and you've found our code helpful, please buy us a round!

Please use, reuse, and modify these files as you see fit. Please maintain attribution to SparkFun Electronics and release anything derivative under the same license.

Distributed as-is; no warranty is given.

- Your friends at SparkFun.
