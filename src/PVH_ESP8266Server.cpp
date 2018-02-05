/**********************************************************************
* VERSION 1.0 / PAULVHA / JANUARY 2018  / ESP8266-PVH-driver
*
* PVH_ESP8266Server.cpp
*
* This an updated version for the ESP8266 AT WIFI shield driver
* for the Arduino. It contains a number of bugfixes, enhancements and
* new features that are not part of the original version (see below)
*
* Distributed as-is; no warranty is given.
***********************************************************************
* Original version :
*
SparkFunESP8266Server.cpp
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
******************************************************************************/

#include "PVH_ESP8266WiFi.h"
#include <Arduino.h>
#include "util/PVH_AT.h"
#include "PVH_ESP8266Server.h"

// initialize the port to use
ESP8266Server::ESP8266Server(uint16_t port)
{
    _port = port;
}

/* create a TCP server connection */
void ESP8266Server::begin()
{
    esp8266.configureTCPServer(_port, 1);
}

/* checks whether a new client has logged on to the server */
ESP8266Client ESP8266Server::available(uint8_t wait)
{
        int retVal;

        // try to read a string like 0,CONNECT
        if (esp8266.readForResponse(",CONNECT", wait) > 0)
        {
            char *p = esp8266.searchBuffer(",CONNECT");
            p -= 1; // Move p back one character

            // ascii to digital
            uint8_t sock = *p - 48;

            ESP8266Client client(sock);
            return client;
        }

        // This is the best way as the information is coming directly
        // from the ESP8266.
        retVal = esp8266.status();
        if (retVal == ESP8266_STATUS_CONNECTED )
        {
            for (int sock=0; sock<ESP8266_MAX_SOCK_NUM; sock++)
            {
                // ESP8266_SOCK_NOT_AVAIL (255)is NOT set and needs to be connected where I am the server.
                // Only the first and only client is ever helped. In the current set up you can
                // not have more connections as a server.
                if ((esp8266._status.ipstatus[sock].linkID != ESP8266_SOCK_NOT_AVAIL) &&
                      (esp8266._status.ipstatus[sock].tetype == ESP8266_SERVER))
                {
                        ESP8266Client client(sock);
                        return client;
                }
            }
        }

        return 0;
}
// never used
uint8_t ESP8266Server::status()
{
        return esp8266.status();
}

// never used
size_t ESP8266Server::write(uint8_t b)
{
    return write(&b, 1);
}
/* does nothing as it is never called. As soon as a client connects
 * the communication is handled by the ESP8266Client().
 */
size_t ESP8266Server::write(const uint8_t *buffer, size_t size)
{

        size_t n = 0;
/*
    for (int sock = 0; sock < MAX_SOCK_NUM; sock++)
    {
        if (WiFiClass::_server_port[sock] != 0)
        {
                ESP8266Client client(sock);

            if (WiFiClass::_server_port[sock] == _port &&
                client.status() == ESTABLISHED)
            {
                return esp8266.tcpSend(sock, buf, size);
            }
        }
    }
*/
    return n;
}
