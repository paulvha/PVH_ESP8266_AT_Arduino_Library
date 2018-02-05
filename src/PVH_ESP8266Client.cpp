/**********************************************************************
* VERSION 1.0 / PAULVHA / JANUARY 2018  / ESP8266-PVH-driver
*
* PVH_ESP8266Client.cpp
*
* This an updated version for the ESP8266 AT WIFI shield driver
* for the Arduino. It contains a number of bugfixes, enhancements and
* new features that are not part of the original version (see below)
*
* Distributed as-is; no warranty is given.
***********************************************************************
* Original version :
*
SparkFunESP8266Client.cpp
ESP8266 WiFi Shield Library Client Source File
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
**********************************************************************/

#include "PVH_ESP8266WiFi.h"
#include <Arduino.h>
#include "util/PVH_AT.h"
#include "PVH_ESP8266Client.h"

/* changed to set _socket as not available
 * as such we when a client connecting to a remote server is made we need
 * to obtain our local socket at the time of connecting
 */
ESP8266Client::ESP8266Client()
{
    _socket = ESP8266_SOCK_NOT_AVAIL;
}

/* Set socket (when called from server.cpp as a client connected)
 *
 * This is a bit confusion: are we now a client connecting to a
 * remote server OR as a remote client connected to the ESP8266 running
 * as a server ?? The connected() has been reworked to handle this.
 */
ESP8266Client::ESP8266Client(uint8_t sock)
{
    _socket = sock;
}

/* get status update from ESP8266. It tells something about
 * status of the WIFI connection, but nothing about the status of
 * the client connected socket. Hence the code in connected() has
 * been updated to handle that differently
 */
uint8_t ESP8266Client::status()
{
    return esp8266.status(1);
}

// add NO timeout
int ESP8266Client::connect(IPAddress ip, uint16_t port)
{
    return connect(ip, port, 0);
}

// add NO timeout
int ESP8266Client::connect(const char *host, uint16_t port)
{
    return connect(host, port, 0);
}

// make constant character from string
int ESP8266Client::connect(String host, uint16_t port, uint32_t keepAlive)
{
    return connect(host.c_str(), port, keepAlive);
}

// make constant character from IP address
int ESP8266Client::connect(IPAddress ip, uint16_t port, uint32_t keepAlive)
{
    char ipAddress[16];
    sprintf(ipAddress, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

    return connect((const char *)ipAddress, port, keepAlive);
}

/*
 * here the real connect call happens
 */
int ESP8266Client::connect(const char* host, uint16_t port, uint32_t keepAlive)
{
    // if socket was not set (during the creation of the handle)
    if(_socket == ESP8266_SOCK_NOT_AVAIL)
        _socket = getFirstSocket();

    // if _socket is available
    if (_socket != ESP8266_SOCK_NOT_AVAIL)
        return esp8266.tcpConnect(_socket, host, port, keepAlive);

    return -1;
}

// write one character
size_t ESP8266Client::write(uint8_t c)
{
    return write(&c, 1);
}

/* the client can only be connected to 1 server */
size_t ESP8266Client::write(const uint8_t *buf, size_t size)
{
    if (_socket != ESP8266_SOCK_NOT_AVAIL)
       return esp8266.tcpSend(_socket, buf, size);

    return ESP8266_CMD_BAD;
}

/* check for the number of available bytes
 * reworked the routine as the original did not make sense
 *
 * This does not check whether the received data if for this client's
 * socket, but assumes that ALL received data is for a single user/socket.
 *
 * The whole socket structure is questionable, also because the
 * Arduino can only run as single user.
 */
int ESP8266Client::available()
{
    if (esp8266.available() == 0)
    {
        // Delay for the amount of time it'd take to receive one character
        // 10 bits assummed, 1e6 = milliseconds
        delayMicroseconds((1 / esp8266._baud) * 10 * 1E6);
    }

    return esp8266.available();
}

// read one byte
int ESP8266Client::read()
{
    return esp8266.read();
}

/* if less bytes than requested size are available return
 * else copy the amount of 'size' into buffer
 *
 * why would you do that and why not call available in client??
 * It is part of the original library , but not called from anywhere ..
 */
int ESP8266Client::read(uint8_t *buf, size_t size)
{
    if (esp8266.available() < size)   return 0;

    for (int i=0; i<size; i++)  buf[i] = esp8266.read();

    return 1;
}

/* check whether there is anything.. */
int ESP8266Client::peek()
{
    return esp8266.peek();
}

/* flush the input /output buffers */
void ESP8266Client::flush()
{
    esp8266.flush();
}

/* close current socket */
void ESP8266Client::stop()
{
    if(_socket != ESP8266_SOCK_NOT_AVAIL)
    {
        esp8266.close(_socket);
        _socket = ESP8266_SOCK_NOT_AVAIL;
    }
}

/*
 * with updateStatus() a array with connected information will be
 * updated by reading from the ESP8266.
 * All entries in the structure will be updated : The linkID will be set
 * to either the _socket number (if still connected with a remote
 * server or client) or ESP8266_SOCK_NOT_AVAIL if NOT connected.
 *
 * return 1 if socket is still connected
 * else 0
 */
uint8_t ESP8266Client::connected()
{
    // first make sure we have a valid socket, else we are done.
    if (_socket == ESP8266_SOCK_NOT_AVAIL)    return 0;

    else     // check that the socket is still alive
    {
        // update status
        esp8266.updateStatus();

        // if socket is still active connected
        if (esp8266._status.ipstatus[_socket].linkID != ESP8266_SOCK_NOT_AVAIL)
            return 1;
    }
    return 0;
}

/* This is the response to the statement "if (client)" in a sketch
 * One would expect that it is a check whether it is larger than zero
 * but as 'client' is a handle, it will check whether it (still) connected
 */
ESP8266Client::operator bool()
{
    return connected();
}

/* Private Methods
 *
 * The whole socket structure is questionable, also because with the
 * Arduino can only run as single user.
 *
 * BUT he ! you never know what the future will bring :-)
 */
uint8_t ESP8266Client::getFirstSocket()
{
    // update the LinkId table directly from the ESP8266
    esp8266.updateStatus();

    for (int i = 0; i < ESP8266_MAX_SOCK_NUM; i++)
    {
        if (esp8266._status.ipstatus[i].linkID == ESP8266_SOCK_NOT_AVAIL)
        {
            return i;
        }
    }
    return ESP8266_SOCK_NOT_AVAIL;
}
