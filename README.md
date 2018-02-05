PVH ESP8266 AT Arduino Library Version 1.0
=============================================
This is an enhanced version of the original SparkFun ESP8266 AT Arduino Library

A large number of bugfixes have been applied + new features that the
ESP8266 supports, but are not implemented in the original libary

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

In the extras-directory you will find a lot of extra information.

The following workbooks.odt are available:

1. workbook_client_loopback
    connect as a client to a remote server. A test-server program for
    Linux (Loop.c) is available in extras-folder.

2. workbook_softAP_loopback
    Create an Access Point server and perform loopback. A test-client
    program for Linux (tclient.c) is available in extras-folder.

3. workbook_softAP_loopback
    Create an Access Point server or server on the local network and
    perform different GPIO function on the Arduino Pins and/or the
    ESP8266 pins. A test-client program for Linux (tclient.c) is
    available in extras-folder.

4. workbook_server_loopback or talkback
    Create a server on the local network and perform talkback
    (basic Chat) or loopback. A test-client program for Linux (tclient.c)
    is available in extras-folder.

##  calls and subroutines

The following coding documentation in the extras-directory :

1. PVH_ESP8266 library commands.odt
    An overview of all the library calls, highlevel describtion and return.

2. available sketch routines.odt:
    An overview of the available Sketch routines and in which examples sketches
    they are available.


Below the original README.md from SparkFun

Paul van Haastrecht
January 2018

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
