/**********************************************************************
* VERSION 1.0 / PAULVHA / JANUARY 2018  / ESP8266-PVH-driver
*
* PVH_ESP8266Client.h
*
* This an updated version for the ESP8266 AT WIFI shield driver
* for the Arduino. It contains a number of bugfixes, enhancements and
* new features that are not part of the original version (see below)
*
* VERSION 2.1 / PAULVHA / November 2018  / ESP8266-PVH-driver
* new option + bug fixes
* - with get_url_parameter you can obtain the query string as part of the URL
* - fixed issue with readData() in different sketches to prevent buffer overrun
* - updated client.connected() to enable more stable performance and capture url_parameters
* - update server.available() to provide additional feedback in seperate variable.
* - change updateStatus() and readForResponses() to capture Url_parameters
*
* Distributed as-is; no warranty is given.
***********************************************************************
* Original version :
*
SparkFunESP8266Client.h
ESP8266 WiFi Shield Library Client Header File
Jim Lindblom @ SparkFun Electronics
Original Creation Date: June 20, 2015
http://github.com/sparkfun/SparkFun_ESP8266_AT_Arduino_Library

!!! Description Here !!!

Development environment specifics:
    IDE: Arduino 1.6.5
    Hardware Platform: Arduino Uno
    ESP8266 WiFi Shield Version: 1.0

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef _SPARKFUNESP8266CLIENT_H_
#define _SPARKFUNESP8266CLIENT_H_

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <IPAddress.h>
#include "Client.h"
#include "PVH_ESP8266WiFi.h"

class ESP8266Client : public Client {

public:
    ESP8266Client();
    ESP8266Client(uint8_t sock);

    uint8_t status();

    virtual int connect(IPAddress ip, uint16_t port);
    virtual int connect(const char *host, uint16_t port);

    int connect(IPAddress ip, uint16_t port, uint32_t keepAlive);
    int connect(String host, uint16_t port, uint32_t keepAlive = 0);
    int connect(const char *host, uint16_t port, uint32_t keepAlive);

    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *buf, size_t size);
    virtual int available();
    virtual int read();
    virtual int read(uint8_t *buf, size_t size);
    virtual int peek();
    virtual void flush();
    virtual void stop();
    virtual uint8_t connected();
    virtual operator bool();
    void extract_parameter();
    uint8_t get_url_parameter(char * buf,uint8_t buf_size);
    friend class WiFiServer;

    using Print::write;

private:
    static uint16_t _srcport;
    uint16_t        _socket;
    bool             ipMuxEn;

    uint8_t getFirstSocket();
};

#endif
