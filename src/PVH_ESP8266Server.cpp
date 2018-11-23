/**********************************************************************
* VERSION 1.0 / PAULVHA / JANUARY 2018  / ESP8266-PVH-driver
*
* PVH_ESP8266Server.cpp
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
#include "printf.h"

// initialize the port to use
ESP8266Server::ESP8266Server(uint16_t port)
{
    _port = port;

    for (int j=0; j<ESP8266_MAX_SOCK_NUM; j++)
    {
       esp8266._status.ipstatus[j].linkID = ESP8266_SOCK_NOT_AVAIL;
    }
    esp8266._status.URL_parameter = ESP8266_NEED_PARAMETER; // obtain URL parameters on next connect
}

/* create a TCP server connection */
void ESP8266Server::begin()
{
    esp8266.configureTCPServer(_port, 1);
}

/* checks whether a new client has logged on to the server
 * result will return the following values
 *   0 and above, the socket-number / linkID that in the client return
 *  -1  NO free scoket entry ( hard to believe)
 *  -2  nothing connected
 *  -3  no WFIF IP ( lost it !)
 *
 *  A check on the return client does not make sense :
 *  e.g. if (client > 0 )
 *
 *  client is NOT a number,but an instance of the client calls/variables
 *  A check like that will cause a client.connected() call
 *
 *  One should check the returned result values first.
 * */
ESP8266Client ESP8266Server::available(uint8_t wait, int *result)
{
        int  i;
        char *p;
        uint8_t sock;

        // try to read a string like 0,CONNECT
        if (esp8266.readForResponse(",CONNECT", wait) > 0)
        {
            p = esp8266.searchBuffer(",CONNECT");
            p -= 1; // Move p back one character

            // ascii to digital
            sock = *p - 48;

            ESP8266Client client(sock);

            if (result != 0) *result = sock;
            return client;
        }

        // If no connect string was detected, tyr to read the current status directly
        // from the ESP8266.

        i = esp8266.status(true);

        // lost connection
        if ( i == ESP8266_STATUS_NOWIFI )
        {
            if (result != 0) *result = -3;
            return 0;
        }

        // no client connected
        if (i != ESP8266_STATUS_CONNECTED )
        {
            *result = -2;
            return 0;
        }

        for (sock=0; sock<ESP8266_MAX_SOCK_NUM; sock++)
        {
            // ESP8266_SOCK_NOT_AVAIL (255) is NOT set and needs to be connected where I am the server.
            // Only the first and only client is ever helped. In the current set up you can
            // not have more connections as a server.
            if ((esp8266._status.ipstatus[sock].linkID != ESP8266_SOCK_NOT_AVAIL) &&
                  (esp8266._status.ipstatus[sock].tetype == ESP8266_SERVER))
            {

                 ESP8266Client client(sock);
                 if (result != 0) *result = sock;
                 return client;
            }
        }
        // no free entry // nothing detected
        if (result != 0) *result = -1;
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
