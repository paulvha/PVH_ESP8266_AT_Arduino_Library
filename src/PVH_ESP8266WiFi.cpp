/**********************************************************************
* VERSION 1.0 / PAULVHA / JANUARY 2018  / ESP8266-PVH-driver
*
* PVH_ESP8266Wifi.cpp
*
* This an updated version for the ESP8266 AT WIFI shield driver
* for the Arduino. It contains a number of bugfixes, enhancements and
* new features that are not part of the original version (see below)
*
* Distributed as-is; no warranty is given.
***********************************************************************
* Original version :
*
SparkFunESP8266WiFi.cpp
ESP8266 WiFi Shield Library Main Source File
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

#include <PVH_ESP8266WiFi.h>
#include <Arduino.h>
#include "util/PVH_AT.h"
#include "PVH_ESP8266Client.h"
#include "printf.h"

#define ESP8266_DISABLE_ECHO

////////////////////////
// Buffer Definitions //
////////////////////////
#ifdef ESP_LIGHT
#define ESP8266_RX_BUFFER_LEN 80           // Number of bytes in the serial receive buffer
#else
#define ESP8266_RX_BUFFER_LEN 200           // Number of bytes in the serial receive buffer
#endif

char esp8266RxBuffer[ESP8266_RX_BUFFER_LEN];// also used for sending to save RAM Bytes
unsigned int bufferHead;                    // Holds position of latest byte placed in buffer.
int ESP_DEBUG = 0;                          // enable debugging messages
////////////////////
// Initialization //
////////////////////

ESP8266Class::ESP8266Class()
{
    // nothing
}

/* set of reset ESP_DEBUG messages from driver
 * 0 = off
 * 1 = display commands
 * 2 = display commands + receive details
 */

bool ESP8266Class::debug(int act)
{
    if (act > 0 || act < 3) ESP_DEBUG = act;
}

bool ESP8266Class::begin(unsigned long baudRate, esp8266_serial_port serialPort)
{
    _baud = baudRate;
    if (serialPort == ESP8266_SOFTWARE_SERIAL)
    {
        swSerial.begin(baudRate);
        _serial = &swSerial;
    }
    else if (serialPort == ESP8266_HARDWARE_SERIAL)
    {
        Serial.begin(baudRate);
        _serial = &Serial;
    }

    // paulvha:
    // extending reset/test as the shield MIGHT still be busy with a
    // previous command.

    if (test() == false)
    {
        reset();

        if(test() == false)
        {
            restore();

            // if test() failed return false
            if(test() == false)
                return false;
        }
    }

    //if (!setTransferMode(0))
    //  return false;

    // set for multi connect. This driver has routines that expect that
    // the ESP8266 has been set to this mode
     if (!setMux(1))
        return false;

    // the shield during  test neglected this !
#ifdef ESP8266_DISABLE_ECHO
    if (!echo(false))
        return false;
#endif
    return true;
}

///////////////////////
// Basic AT Commands //
///////////////////////

// paulvha:
// read untill all data has been read from the serial
void ESP8266Class::rx_empty(void)
{
    while (_serial->available() > 0)  _serial->read();
}


bool ESP8266Class::test()
{
    sendCommand(ESP8266_TEST); // Send AT

    if (readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT) > 0)
        return true;

    return false;
}

/* reset the board */
int ESP8266Class::reset()
{
    sendCommand(ESP8266_RESET); // Send AT+RST

    // give time to settle
    delay(2000);

    // OK is the immediate response by READY when action is complete
    return readForResponse(RESPONSE_READY, COMMAND_RESET_TIMEOUT);
}

/* restore start values */
int ESP8266Class::restore()
{
    sendCommand(ESP8266_RESTORE); // Send AT+RESTORE

    // give time to settle
    delay(2000);

    // OK is the immediate response by READY when action is complete
    return readForResponse(RESPONSE_READY, COMMAND_RESET_TIMEOUT);
}

/* echo the commands back or not
 * This is NOT consistent implemented in the firmware
 */
bool ESP8266Class::echo(bool enable)
{
    if (enable)
        sendCommand(ESP8266_ECHO_ENABLE);
    else
        sendCommand(ESP8266_ECHO_DISABLE);

    if (readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT) > 0)
        return true;

    return false;
}

// backward compatible
bool ESP8266Class::setBaud(unsigned long baud)
{
    return (setBaud(CUR, baud));
}

// set UART baudrate
bool ESP8266Class::setBaud(bool mode, unsigned long baud)
{
// in case light version was requested to save RAM
#ifdef ESP_LIGHT
    return -1;
#else
    char parameters[16];
    memset(parameters, 0, 16);

    // Constrain parameters:
    baud = constrain(baud, 110, 115200);

    // Put parameters into string
    sprintf(parameters, "%d,8,1,0,0", baud);

    // Send AT+UART=baud,databits,stopbits,parity,flowcontrol
    if (mode == CUR)
        sendCommand(ESP8266_UART_CUR, ESP8266_CMD_SETUP, parameters);
    else
        sendCommand(ESP8266_UART_DEF, ESP8266_CMD_SETUP, parameters);

    if (readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT) > 0)
        return true;

    return false;
#endif
}

int16_t ESP8266Class::getVersion(char *ATversion, char *SDKversion, char *compileTime)
{
// in case light version was requested to save RAM
#ifdef ESP_LIGHT
    return -1;
#else
    sendCommand(ESP8266_VERSION); // Send AT+GMR
    // Example Response: AT version:0.30.0.0(Jul  3 2015 19:35:49)\r\n (43 chars)
    //                   SDK version:1.2.0\r\n (19 chars)
    //                   compile time:Jul  7 2015 18:34:26\r\n (36 chars)
    //                   OK\r\n
    // (~101 characters). Do not be surprised if you run in problems.. see remarks
    // as detectAP()

    // Look for "OK":
    int16_t rsp = (readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT) > 0);
    if (rsp > 0)
    {
        char *p, *q;
        // Look for "AT version" in the rxBuffer
        p = strstr(esp8266RxBuffer, "AT version:");
        if (p == NULL) return ESP8266_RSP_UNKNOWN;
        p += strlen("AT version:");
        q = strchr(p, '\r'); // Look for \r
        if (q == NULL) return ESP8266_RSP_UNKNOWN;
        strncpy(ATversion, p, q-p);

        // Look for "SDK version:" in the rxBuffer
        p = strstr(esp8266RxBuffer, "SDK version:");
        if (p == NULL) return ESP8266_RSP_UNKNOWN;
        p += strlen("SDK version:");
        q = strchr(p, '\r'); // Look for \r
        if (q == NULL) return ESP8266_RSP_UNKNOWN;
        strncpy(SDKversion, p, q-p);

        // Look for "compile time:" in the rxBuffer
        p = strstr(esp8266RxBuffer, "compile time:");
        if (p == NULL) return ESP8266_RSP_UNKNOWN;
        p += strlen("compile time:");
        q = strchr(p, '\r'); // Look for \r
        if (q == NULL) return ESP8266_RSP_UNKNOWN;
        strncpy(compileTime, p, q-p);
    }

    return rsp;
#endif
}

/* set sleep mode for ESP8266
 *
 * mode:
 *  ESP_SLEEP_DISABLE = 0,
 *  ESP_SLEEP_LIGHT = 1,
 *  ESP_SLEEP_MODEM = 2
 *
 * A good article about sleep and the differences is found on
 * https://www.losant.com/blog/making-the-esp8266-low-powered-with-deep-sleep
 *
 * Also in the extra directory there is a development document
 * with more information
 *
 * Modem-sleep mode is enabled only when ESP8266 connects to the
 * router in station mode.
 */
int16_t ESP8266Class::sleep(int mode)
{
// in case light version was requested to save RAM
#ifdef ESP_LIGHT
    return -1;
#else

    if (mode != ESP_SLEEP_DISABLE && mode != ESP_SLEEP_LIGHT &&
    mode != ESP_SLEEP_MODEM)  return ESP8266_CMD_BAD;

    sendCommand(ESP8266_SLEEP, ESP8266_CMD_SETUP, mode);

    return readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
#endif
}

/* set deep-sleep mode for ESP8266
 * SleepTime  : time in milliseconds
 *
 * deepsleep can be maximum 71 minutes
 *
 * ///////////////  DEEPSLEEP needs hardware wire ////////////////////////
 * The RST pin is held at a HIGH signal while the ESP8266 is running.
 * However, when the RST pin receives a LOW signal, it restarts the microcontroller.
 * Once your device is in Deep-sleep, it will send a LOW signal to GPIO 16
 * when the sleep timer is up. You need to connect GPIO 16 to RST to wake up
 * ( or reset ) the device when Deep-sleep is over.
 *
 * GPIO16 is connected to XP0 in the ESP8266 GPIO output connector on
 * the SparkFun ES8266 AT WIFI SHELD. In order for deep sleep to work make a
 * connection between RST and XP0 on the GPIO connector from the ESP8266.
 */

int16_t ESP8266Class::Deepsleep(uint32_t SleepTime)
{
// in case light version was requested to save RAM
#ifdef ESP_LIGHT
    return -1;
#else

    char timeChar[12];

    sprintf(timeChar, "%d", SleepTime);

    sendCommand(ESP8266_DEEP_SLEEP, ESP8266_CMD_SETUP, timeChar);

    return readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
#endif
}


/* set TXPOWER mode for ESP8266
 *
 * MODE SET : will set the TX power in steps of 0,25db between 0 and 82
 *
 * Be carefull setting the level too high with the small on-baord antenna
 * as it will not work all the time and take more power.
 *
 * mode VDD33 : this will the power according to the VCC. Tout can not
 * be used however and must have been disabled during compiling the GPIO
 * ///////////////////////////////////////////////////////////////////
 * ///////////////////// VDD33 mode has been removed /////////////////
 * ///////////////////////////////////////////////////////////////////
 * The value after reading is always 65535. Which seems to indicate that
 * TOUT was NOT disabled, however there is also NO ability access the ADC
 * through GPIO calls. (option 2)
 * The ADC as such is NOT accessable on the ESP8266 WIFi Sparkfun board.
 *
 */
int16_t ESP8266Class::txpower(int mode, int level)
{
    /*if (mode == ESP8266_POWER_VDD33)
    {
       if (level < ESP8266_POWER_VDD33_MIN || level > ESP8266_POWER_VDD33_MAX )
            return ESP8266_CMD_BAD;

       sendCommand(ESP8266_TXVDD, ESP8266_CMD_SETUP, level);
    }
    */

    if (mode == ESP8266_POWER_SET)
    {
        if (level < ESP8266_POWER_MIN || level > ESP8266_POWER_MAX )
            return ESP8266_CMD_BAD;

        sendCommand(ESP8266_TXPOWER, ESP8266_CMD_SETUP, level);
    }

    return readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
}


////////////////////
// WiFi Functions //
////////////////////

// backward compatibility
int16_t ESP8266Class::getMode()
{
    return getMode(CUR);
}

/* getMode()
 * Input:
 * stat : CUR for current or DEF for default
 *
 * Output:
 *    - Success: 1, 2, 3 (ESP8266_MODE_STA, ESP8266_MODE_AP, ESP8266_MODE_STAAP)
 *    - Fail: <0 (esp8266_cmd_rsp)
 */
int16_t ESP8266Class::getMode(bool stat)
{
    if (stat == CUR)
        sendCommand(ESP8266_WIFI_MODE_CUR, ESP8266_CMD_QUERY);
    else
        sendCommand(ESP8266_WIFI_MODE_DEF, ESP8266_CMD_QUERY);

    // Example response: \r\nAT+CWMODE_CUR?\r+CWMODE_CUR:2\r\n\r\nOK\r\n
    // Look for the OK:
    int16_t rsp = readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
    if (rsp > 0)
    {
        // Then get the number after ':':
        char * p = strchr(esp8266RxBuffer, ':');
        if (p != NULL)
        {
            char mode = *(p+1);
            if ((mode >= '1') && (mode <= '3'))
                return (mode - 48); // Convert ASCII to decimal
        }

        return ESP8266_RSP_UNKNOWN;
    }

    return rsp;
}

// backward compatibility
int16_t ESP8266Class::setMode(esp8266_wifi_mode mode)
{
    setMode(CUR,mode);
}

/* setMode()
 *  Input:
 *  stat : CUR or DEF;
 *  mode: 1, 2, 3 (ESP8266_MODE_STA, ESP8266_MODE_AP, ESP8266_MODE_STAAP)
 *  Output:
 *    - Success: >= 0
 *    - Fail: <0 (esp8266_cmd_rsp)
 */

int16_t ESP8266Class::setMode(bool stat, esp8266_wifi_mode mode)
{
    if (stat == CUR)
        sendCommand(ESP8266_WIFI_MODE_CUR, ESP8266_CMD_SETUP, (int) mode);
    else
        sendCommand(ESP8266_WIFI_MODE_DEF, ESP8266_CMD_SETUP, (int) mode);

    return readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
}


// backward compatibility
int16_t ESP8266Class::connect(const char * ssid, const char * pwd)
{
    return connect(CUR, ssid, pwd);
}

// backward compatibility + add current
int16_t ESP8266Class::connect(const char * ssid)
{
    return connect(CUR, ssid, "");
}

/* connect()
 * Input: ssid and pwd const char's
 * Output:
 *    - Success: >0
 *    - Fail: <0 (esp8266_cmd_rsp)
 */
int16_t ESP8266Class::connect(bool mode, const char * ssid, const char * pwd)
{
    clearBuffer();

    if (pwd != NULL)
        sprintf(esp8266RxBuffer,"\"%s\",\"%s\"",ssid,pwd);
    else
        sprintf(esp8266RxBuffer,"\"%s\"",ssid);

    // either current or default
    if (mode == CUR)
        sendCommand(ESP8266_CONNECT_AP_CUR, ESP8266_CMD_SETUP, esp8266RxBuffer);
    else
        sendCommand(ESP8266_CONNECT_AP_DEF, ESP8266_CMD_SETUP, esp8266RxBuffer);

    return readForResponses(RESPONSE_OK, RESPONSE_FAIL, WIFI_CONNECT_TIMEOUT);
}


/* enable or disable autoconnect at power on
 * default is enable
 *
 * action ESP8266_DISABLE or ESP8266_ENABLE
 *
 * return :
 *  OK      :> 0
 *  error   : 0 <=
 */
int16_t ESP8266Class::AutoConnect(int action)
{

// in case light version was requested to save RAM
#ifdef ESP_LIGHT
    return -1;
#else
    sendCommand(ESP8266_AUTO_CONNECT, ESP8266_CMD_SETUP, action);// Send "AT+CWAUTOCONN= 1 or 0"

    return readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
#endif
}

// backward compatibility is current
int16_t ESP8266Class::getAP(char * ssid)
{
    return getAP(CUR,ssid);
}

/* getAP() : get connected Access point name
 * Input:
 *      mode :  CUR = CURRENT or DEF = DEFAULT
 * Output:
 * < 0 = error
 *   0 = not found
 *   1= found and in SSID connected AP
 */

int16_t ESP8266Class::getAP(bool mode, char * ssid)
{
    char    *p;

    if (mode == CUR)
        sendCommand(ESP8266_CONNECT_AP_CUR, ESP8266_CMD_QUERY); // Send "AT+CWJAP_CUR?"
    else
        sendCommand(ESP8266_CONNECT_AP_DEF, ESP8266_CMD_QUERY); // Send "AT+CWJAP_DEF?"

    int16_t rsp = readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);

    // Example Responses: No AP\r\n\r\nOK\r\n
    // - or -
    // +CWJAP_CUR:"WiFiSSID","00:aa:bb:cc:dd:ee",6,-45\r\n\r\nOK\r\n
    if (rsp > 0)
    {
        // Look for "No AP"
        if (strstr(esp8266RxBuffer, "No AP") != NULL)
            return 0;

        // Look for "+CWJAP_CUR:" or "+CWJAP_DEF:"
        // we have to hardcode this string as the command is also echo, despite ATE0
        p = strstr(esp8266RxBuffer, ":");

        if (p != NULL)
        {
            p += 2;
            char * q = strchr(p, '"');
            if (q == NULL) return ESP8266_RSP_UNKNOWN;
            strncpy(ssid, p, q-p); // Copy string to temp char array:
            return 1;
        }

        return 0;
    }

    return rsp;
}

// TODO 2 x time OK and WIFI DISCONNECT ???
int16_t ESP8266Class::disconnect()
{
    sendCommand(ESP8266_DISCONNECT); // Send AT+CWQAP
    // Example response: \r\n\r\nOK\r\nWIFI DISCONNECT\r\n
    // "WIFI DISCONNECT" comes up to 500ms _after_ OK.
    int16_t rsp = readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);

    if (rsp > 0)
    {
        rsp = readForResponse("WIFI DISCONNECT", COMMAND_RESPONSE_TIMEOUT);
        if (rsp > 0)
            return rsp;
        return 1;
    }

    return rsp;
}

/* status()
 * Input:
 *       status_new: 1 set returns
 *
 *       Output:
 *          Success: 2, 3, 4, or 5 (ESP8266_STATUS_GOTIP, ESP8266_STATUS_CONNECTED, ESP8266_STATUS_DISCONNECTED, ESP8266_STATUS_NOWIFI)
 *          - Fail: <0 (esp8266_cmd_rsp)
 *
 *       status_new: 0 set returns
 *          connected : 1
 *          not connected : 0
 *          Fail < 0
 *
 * paulvha : changed this to original intent and otherwise a server setup
 * can not cope with it. Actually this does not say ANYTHING about the status
 * of the connected client sockets in case of server or softAP. Hence the
 * connected() in the client.cpp has been changed.
 */
int16_t ESP8266Class::status(bool status_new)
{
    int16_t statusRet = updateStatus();

    if(status_new)
    {
        if (statusRet > 0) return(_status.stat);
        return statusRet;
    }

    // this is the old status for backward compatibility
    if (statusRet > 0)
    {
        switch (_status.stat)
        {
        case ESP8266_STATUS_GOTIP:        // 2
        case ESP8266_STATUS_DISCONNECTED: // 4 - "Client" disconnected, not wifi
            return 1;
            break;
        case ESP8266_STATUS_CONNECTED:     // 3 Connected, but haven't gotten an IP
        case ESP8266_STATUS_NOWIFI:        // 5 No WiFi configured (also when in softAP
            return 0;
            break;
        }
    }
    return statusRet;
}

/* get TCP status update and store result in internal structure
 * _status.ipstatus:
 *
 *  2 GOTIP
 *  3 CONNECTED
 *  4 DISCONNECT
 *  5 NO WIFI       (returned by softAp all the time (if with valid socket)
 *
 *  It also catchesfor each channel / LinkID (CIPSTATUS)
 *
 * _status.ipstatus[linkId].linkID   : the linkID number (a bit overdone..)
 * _status.ipstatus[linkId].type     : ESP8266_TCP or ESP8266_UDP or unknown
 * _status.ipstatus[linkId].remoteIP : the remote IP
 * _status.ipstatus[linkId].tetype   : I am ESP8266_CLIENT or ESP8266_SERVER;
 *
 * This status table is important for client connections
 *
 */
int16_t ESP8266Class::updateStatus()
{
    sendCommand(ESP8266_TCP_STATUS); // Send AT+CIPSTATUS\r\n
    // Example response: (connected as client)
    // STATUS:3\r\n
    // +CIPSTATUS:0,"TCP","93.184.216.34",80,0\r\n\r\nOK\r\n
    //
    // - or - (clients connected to ESP8266 server)
    // STATUS:3\r\n
    // +CIPSTATUS:0,"TCP","192.168.0.100",54723,1\r\n
    // +CIPSTATUS:1,"TCP","192.168.0.101",54724,1\r\n\r\nOK\r\n
    //
    // - or - (clients connected to ESP8266 SOFT AP)
    // STATUS:5\r\n
    // +CIPSTATUS:0,"TCP","192.168.0.100",54723,1\r\n
    // +CIPSTATUS:1,"TCP","192.168.0.101",54724,1\r\n\r\nOK\r\n
    int16_t rsp = readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);

    if (rsp > 0)
    {
        char * p = searchBuffer("STATUS:");

        if (p == NULL)
            return ESP8266_RSP_UNKNOWN;

        p += strlen("STATUS:");

        // ascii to decimal ( - 0)
        _status.stat = (esp8266_connect_status)(*p - 48);

        // check other connections
        for (int i=0; i<ESP8266_MAX_SOCK_NUM; i++)
        {
            p = strstr(p, "+CIPSTATUS:");

            if (p == NULL)
            {
                // Didn't find any extra CIPSTATUS'. Set (other) linkID to ESP8266_SOCK_NOT_AVAIL.
                for (int j=i; j<ESP8266_MAX_SOCK_NUM; j++)
                    _status.ipstatus[j].linkID = ESP8266_SOCK_NOT_AVAIL;

                return rsp;
            }
            else
            {
                p += strlen("+CIPSTATUS:");

                // Find linkID:
                uint8_t linkId = *p - 48;

                // can't be more than maximum sockets
                if (linkId >= ESP8266_MAX_SOCK_NUM)
                    return rsp;

                // looks a bit overdone..but indicates the channel is still in us
                _status.ipstatus[linkId].linkID = linkId;

                // Find type (p pointing at linkID):
                p += 3; // Move p to either "T" or "U"
                if (*p == 'T')
                    _status.ipstatus[linkId].type = ESP8266_TCP;
                else if (*p == 'U')
                    _status.ipstatus[linkId].type = ESP8266_UDP;
                else
                    _status.ipstatus[linkId].type = ESP8266_TYPE_UNDEFINED;

                // Find remoteIP (p pointing at first letter or type):
                p += 6; // Move p to first digit of first octet.
                for (uint8_t j = 0; j < 4; j++)
                {
                    char tempOctet[4];
                    memset(tempOctet, 0, 4); // Clear tempOctet

                    size_t octetLength = strspn(p, "0123456789"); // Find length of numerical string:

                    strncpy(tempOctet, p, octetLength); // Copy string to temp char array:
                    _status.ipstatus[linkId].remoteIP[j] = atoi(tempOctet); // Move the temp char into IP Address octet

                    p += (octetLength + 1); // Increment p to next octet
                }

                // Find port (p pointing at ',' between IP and port:
                p += 1; // Move p to first digit of port
                char tempPort[6];
                memset(tempPort, 0, 6);
                size_t portLen = strspn(p, "0123456789"); // Find length of numerical string:
                strncpy(tempPort, p, portLen);
                _status.ipstatus[linkId].port = atoi(tempPort);
                p += portLen + 1;

                // Find tetype (p pointing at tetype)
                if (*p == '0')
                    _status.ipstatus[linkId].tetype = ESP8266_CLIENT;
                else if (*p == '1')
                    _status.ipstatus[linkId].tetype = ESP8266_SERVER;
            }
        }
    }

    return rsp;
}

/* Paulvha :
 * listAP() : list all detected AP's
 *
 * Input:
 *  out     : buffer to store SSID's
 *  len     : length of buffer
 *  timeout : max time to wait for 'OK'
 *
 *  Output:
 *
 *    - Success:
 *              Received information from ESP866x wifi in out
 *              return # bytes stored in out.
 *
 *    - Fail: 0
 *
 * Although with the command  AT+CWLAP you can obtain the information
 * from ALL the SSID the ESP866x discovered, within Softserial the standard
 * receivebuffer is 64.
 *
 * Defined in arduino-nightly/hardware/arduino/avr/libraries/SoftSerial/src/SoftwareSerial.h
 *  #define _SS_MAX_RX_BUFF 64
 *
 * That is TOO small and it will overrun. WHY ? well SoftwareSerial is interrupt driven,
 * as soon as the input level changes, disables that interrupt, trigger a carefull
 * time-based checking on the input level to capture the 8 bits, store the byte and enable interrupt.
 * This is handled by the SAME processor as the code below, which will get hardly time
 * to read from that buffer. As such with the call to get ALL SSID, the receivebuffer will
 * be full in no time, causing the program to get lost. Extending the buffer to 256 will
 * enable to capture more input, but whether it will overrun is depending on the amount
 * of detected SSID. The buffer could be extended more, but it will take more RAM space.
 *
 * It is better to use detectAP() to check whether a certain SSID is discovered OR extend the
 * softserial buffer size
 */

int ESP8266Class::listAP(char *out, int len, unsigned long timeout )
{
// in case light version was requested to save RAM
#ifdef ESP_LIGHT
    return -1;
#else

    int start = 0, end = 0;             // start to grep SSID, end to grep OK
    unsigned long timeIn = millis();    // Timestamp coming into function
    unsigned int received = 0;          // received keeps track of number of chars read
    char c;

    // reset buffer
    memset(out, 0x0, len);

    sendCommand(ESP8266_LIST_AP); // Send AT+CWLAP\r\n

    /* Example Response:
      + C W L A P : ( 0 , " h o m e l a n - g u e s t " , - 5 4 , " c 0 : c 1 : c 0 : 5 8 : c a : e 5 " , 1 , 3 6 ) \r\n
      + C W L A P :  ( 4 , " h o m e l a n " , - 5 6 , " c 0 : c 1 : c 0 : 5 8 : c a : e 3 " , 1 , 3 8 ) \r\n
      + C W L A P :  ( 3 , " Z i g g o 4 7 6 1 B F 1 " , - 9 2 , " 5 4 : 6 7 5 e : 5 4 " 1 5 ) \r\n
    */

    // Look for the OK or timeout
    // give longer delay to allow for processing
    while (timeIn + timeout - COMMAND_RESPONSE_TIMEOUT > millis()) // While we haven't timed out
    {
        if (_serial->available()) // If data is available on UART RX
        {
            c = _serial->read();

            // do not overrun output buffer
            if (received < len)  out[received++] = c;

            // look for OK (implemented this way for increasing speed)
            if (end == 0)
            {
                if (c == 'O') end = 1;
            }
            else if (end == 1)
            {
                if (c == 'K') return received;
                else end = 0;
            }
        }
    }

    return received;
#endif
}


/* Paulvha :
 * remarks at listAP()
 * this will try to detect a single SSID
 * Input:
 *     ssid : ssid name to look for (max 20 characters)
 *      out : buffer to return information (0x0 terminated)
 *      len : maximum length return buffer
 *  timeout : timeout in milliseconds
 *
 * return
 *  ERROR
 *       -1 no characters received (timeout)
 *       -2 character received but did not receive OK
 *              Buffer can be read for debug
 *       -4 incorrect length SSID (0 <> 20)
 *       -5 return buffer is to small
 *              Buffer can be read but last byte that fitted is overwritten with 0x0 for debug
 *       -6 incorrect answer received
 *
 * OK     number of received bytes in out-buffer
 */
int ESP8266Class::detectAP(char *ssid, char *out, int len, unsigned long timeout)
{
    char buf[23];
    int i;

    if (strlen(ssid) == 0 || strlen(ssid) > 20) return(-4);

    // clear buffer
    memset(buf, 0x0, sizeof(buf));

    sprintf(buf, "\"%s\"", ssid);

    // Send AT+CWLAP="SSID"
    sendCommand(ESP8266_LIST_AP, ESP8266_CMD_SETUP, buf);

    // Look for the OK:
    int16_t rsp = readForResponse(RESPONSE_OK, timeout);

    // + CWLAP:(0,"homelan",-54,"c0:c1:c0:58:ca:e5",1,36)\r\nOK\r\n
    if (rsp > 0)
    {
            // Look for "+CWLAP:"
            // the search string has been hardcoded as the ESP8266 does
            // echo the command back despite ATE0 !
            char * p = strstr(esp8266RxBuffer, "AP:");

            if (p != NULL)
            {
                // skip AP:
                p +=  3;

                if (strchr(p, ')') == NULL) return -6;

                // copy to output buffer
                i=0;
                while(*p != '\r' && i < len)  out[i++]= *p++;

                // terminate
                if (i < len)
                {
                    out[i] = 0x0;
                    return i;
                }
                else
                {   // buffer not large enough overwrite last byte

                    out[i-1] = 0x0;
                    return (-5);
                }
            }
            else
                return (-6);
    }

    else if (rsp == ESP8266_RSP_UNKNOWN)
    {
            // we got something BUT NOT OK ... copy for debug
            for (i=0 ; i < bufferHead && i < len; i++) out[i]=esp8266RxBuffer[i];

            // terminate
            if (i < len) out[i] = 0x0;
            else  out[i-1] = 0x0;
    }

    return(rsp);
}

/* localIP()
 *
 * Input: none
 * Output:
 *    - Success: Device's local IPAddress
 *    - Fail: 0
 *
 * Do not use with SoftAP
 */
IPAddress ESP8266Class::localIP()
{

    sendCommand(ESP8266_GET_LOCAL_IP); // Send AT+CIFSR\r\n
    // Example Response: +CIFSR:STAIP,"192.168.0.114"\r\n
    //                   +CIFSR:STAMAC,"18:fe:34:9d:b7:d9"\r\n
    //                   \r\n
    //                   OK\r\n
    // Look for the OK:
    int16_t rsp = readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);

    if (rsp > 0)
    {
        // Look for "STAIP" in the rxBuffer
        char * p = strstr(esp8266RxBuffer, "STAIP");
        if (p != NULL)
        {
            IPAddress returnIP;

            p += 7; // Move p seven places. (skip STAIP,")
            for (uint8_t i = 0; i < 4; i++)
            {
                char tempOctet[4];
                memset(tempOctet, 0, 4); // Clear tempOctet

                size_t octetLength = strspn(p, "0123456789"); // Find length of numerical string:
                if (octetLength >= 4) // If it's too big, return an error
                    return ESP8266_RSP_UNKNOWN;

                strncpy(tempOctet, p, octetLength); // Copy string to temp char array:
                returnIP[i] = atoi(tempOctet); // Move the temp char into IP Address octet

                p += (octetLength + 1); // Increment p to next octet
            }

            return returnIP;
        }
    }

    return rsp;
}
/* get local MAC */

// backward compatibility
int16_t ESP8266Class::localMAC(char * mac)
{
    return localMAC(CUR, mac);
}

// get local mac (either CURRENT of DEFAULT)
int16_t ESP8266Class::localMAC(bool mode, char * mac)
{
    char    *p;

    if (mode == CUR)
        sendCommand(ESP8266_GET_STA_MAC_CUR, ESP8266_CMD_QUERY); // Send "AT+CIPSTAMAC_CUR?"
    else
        sendCommand(ESP8266_GET_STA_MAC_DEF, ESP8266_CMD_QUERY); // Send "AT+CIPSTAMAC_DEF?"

    int16_t rsp = readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);

    if (rsp > 0)
    {
        // Look for "+CIPSTAMAC_CUR:" or "+CIPSTAMAC_DEF:"
        // despite telling NOT to echo the command.. it does
        if (mode == CUR)
            p = strstr(esp8266RxBuffer, ":");
        else
            p = strstr(esp8266RxBuffer, ":");

        if (p != NULL)
        {
            p += 2;
            char * q = strchr(p, '"');
            if (q == NULL) return ESP8266_RSP_UNKNOWN;
            strncpy(mac, p, q - p); // Copy string to temp char array:
            return 1;
        }
    }

    return rsp;
}

/* SET local (static) IP address (either CUR = CURRENT of DEF = DEFAULT)
 *
 * param IP      : "192.168.6.100"
 * param gateway : "192.168.6.1"
 * param mask    : "255.255.255.0"
 *
 * This configuration interacts with AT+CWDHCP related AT commands:
 *
 * • If enable static IP, DHCP will be disabled;
 * • If enable DHCP, static IP will be disabled;
 * • This will depend on the last configuration.
 *
 */

int16_t ESP8266Class::setlocalIP(bool mode, char * IP, char *gateway, char *mask)
{

    if (strlen(IP) == 0 || strlen(gateway) == 0 || strlen(mask) == 0)
        return ESP8266_CMD_BAD;

    clearBuffer();

    sprintf(esp8266RxBuffer,"\"%s\",\"%s\",\"%s\"",IP,gateway,mask);

    if (mode == CUR)
        sendCommand(ESP8266_SET_STA_IP_CUR, ESP8266_CMD_SETUP, esp8266RxBuffer);
    else
        sendCommand(ESP8266_SET_STA_IP_DEF, ESP8266_CMD_SETUP, esp8266RxBuffer);

    return readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
}

/* Set local mac (either CURRENT of DEFAULT)
 * param mode = CUR or DEF
 * param mac = MAC to set
 */
int16_t ESP8266Class::setlocalMAC(bool mode, char * mac)
{
// in case light version was requested to save RAM
#ifdef ESP_LIGHT
    return -1;
#else
    if (strlen(mac) == 0) return ESP8266_CMD_BAD;

    clearBuffer();

    sprintf(esp8266RxBuffer,"\"%s\"",mac);
    if (mode == CUR)
        sendCommand(ESP8266_GET_STA_MAC_CUR,ESP8266_CMD_SETUP, esp8266RxBuffer); // Send "AT+CIPSTAMAC_CUR="
    else
        sendCommand(ESP8266_GET_STA_MAC_DEF, ESP8266_CMD_SETUP, esp8266RxBuffer); // Send "AT+CIPSTAMAC_DEF="

    int16_t rsp = readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
#endif
}

////////////////////////////////
//      SOFTAP commands       //
////////////////////////////////

// backward compatibility
int16_t ESP8266Class::softAP(const char * ssid, const char * pwd)
{
    return SoftConfigureAP(CUR,ssid,pwd,2,4);
}

/* Configure SoftAP mac (either CUR = CURRENT of DEF = DEFAULT)
 * param ssid    : ssid to set
 * param pwd     : password to set
 * param channel : between channel 0 and 11
 * param encoding: type of password encoding
 *
 * return:
 * OK > 0
 * error <= 0
 */
int16_t ESP8266Class::SoftConfigureAP(bool mode, const char * ssid, const char * pwd, int channel, int encoding)
{
    /*
     * 0 OPEN
     * 2 WPA_PSK
     * 3 WPA2_PSK
     * 4 WPA_WPA2_PSK
     * ESP8266 softAP don’t support WEP.
     */
    if (encoding < 0 || encoding > 4) return ESP8266_CMD_BAD;

    // America supports up to 11, rest of world up to 13
    if (channel < 0 || channel > 11)  return ESP8266_CMD_BAD;

    if (strlen(ssid) == 0) return ESP8266_CMD_BAD;

    clearBuffer();

    sprintf(esp8266RxBuffer,"\"%s\",\"%s\",%d,%d",ssid,pwd,channel,encoding);

    if (mode == CUR)
        sendCommand(ESP8266_AP_CONFIG_CUR, ESP8266_CMD_SETUP, esp8266RxBuffer);
    else
        sendCommand(ESP8266_AP_CONFIG_DEF, ESP8266_CMD_SETUP, esp8266RxBuffer);

    return readForResponses(RESPONSE_OK, RESPONSE_FAIL, WIFI_CONNECT_TIMEOUT);
}

/* Set local mac (either CURRENT of DEFAULT)
 * param mode = CUR or DEF
 * param mac = MAC to set : "5e:cf:7f:e3:7F:11"
 *
 * THE SOFT MAC MUST BE UNIQUE ON THE NETWORK. CAN NOT BE THE SAME
 * AS THE CURRENT HARDWARE MAC. OF THE ESP8266
 *
 */
int16_t ESP8266Class::SoftSetLocalMAC(bool mode, char * mac)
{
// in case light version was requested to save RAM
#ifdef ESP_LIGHT
    return -1;
#else

    if (strlen(mac) == 0) return ESP8266_CMD_BAD;

    clearBuffer();

    sprintf(esp8266RxBuffer,"\"%s\"",mac);

    if (mode == CUR)
        sendCommand(ESP8266_SET_AP_MAC_CUR,ESP8266_CMD_SETUP, esp8266RxBuffer); // Send "AT+CIPSTAMAC_CUR="
    else
        sendCommand(ESP8266_SET_AP_MAC_DEF, ESP8266_CMD_SETUP, esp8266RxBuffer); // Send "AT+CIPSTAMAC_DEF="

    return readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
#endif
}

/* SET local (static) IP address for AP (either CUR = CURRENT of DEF = DEFAULT)
 *
 * param IP      : "192.168.6.100"
 * param gateway : "192.168.6.1"
 * param mask    : "255.255.255.0"
 *
 * This configuration interacts with AT+CWDHCP related AT commands:
 *
 * • If enable static IP, DHCP will be disabled;
 * • If enable DHCP, static IP will be disabled;
 * • This will depend on the last configuration.
 *
 * Only after ESP8266 station connected to AP, station IP can be got
 * and inquiried. This configuration will store in Flash user parameter area.
 */

int16_t ESP8266Class::SoftSetLocalIP(bool mode, char * IP, char *gateway, char *mask)
{
// in case light version was requested to save RAM
#ifdef ESP_LIGHT
    return -1;
#else

    if (strlen(IP) == 0 || strlen(gateway) == 0 || strlen(mask) == 0)
        return ESP8266_CMD_BAD;

    clearBuffer();

    sprintf(esp8266RxBuffer,"\"%s\",\"%s\",\"%s\"",IP,gateway,mask);

    if (mode == CUR)
        sendCommand(ESP8266_SET_AP_IP_CUR, ESP8266_CMD_SETUP, esp8266RxBuffer);
    else
        sendCommand(ESP8266_SET_AP_IP_DEF, ESP8266_CMD_SETUP, esp8266RxBuffer);

    return readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
#endif
}


/* get DHCP status
 *
 *  Bit0 : 0 - soft-AP DHCP disabled
 *         1 - soft-AP DHCP enabled
 *  bit1 : 0 - station DHCP disabled
 *         1 - station DHCP enabled
 *
 *  Return 0 = BOTH Soft-AP and Station DHCP disabled
 *         1 = soft-AP enabled
 *         2 = Station DHCP enabled
 *         3 = BOTH Soft-AP and Station DHCP enabled
 */
int16_t ESP8266Class::GetStatusDHCP(bool mode)
{
    char    *p;

    if (mode == CUR)
        sendCommand(ESP8266_DHCP_EN_CUR, ESP8266_CMD_QUERY); // Send "AT+CWDHCP_CUR?"
    else
        sendCommand(ESP8266_DHCP_EN_DEF, ESP8266_CMD_QUERY); // Send "AT+CWDHCP_DEF?"

    int16_t rsp = readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);

    // Example Responses: No AP\r\n\r\nOK\r\n
    // - or -

    if (rsp > 0)
    {

        // Look for "+CWDHCP_CUR" or "+CWDHCP_DEF"
        if (mode == CUR)
            p = strstr(esp8266RxBuffer, ESP8266_DHCP_EN_CUR);
        else
            p = strstr(esp8266RxBuffer, ESP8266_DHCP_EN_DEF);

        if (p != NULL)
        {
            p += strlen(ESP8266_CONNECT_AP_CUR) + 2;
            return(*p);

        }
    }

    return -1;
}


/* enable or disable DHCP server
 *
 *
 * station 0 = Soft-AP DHCP                 ESP8266_DHCP_SOFTAP
 *         1 = Station DHCP                 ESP8266_DHCP_STATION
 *         2 = soft-AP and Station DHCP     ESP8266_DHCP_BOTH
 *
 * action 0 = disable                       ESP8266_DISABLE
 *        1 = enable                        ESP8266_ENABLE
 */

int16_t ESP8266Class::SetDHCP(bool mode, int station, int action)
{
    clearBuffer();

    sprintf(esp8266RxBuffer,"%d,%d",station,action);

    if (mode == CUR)
        sendCommand(ESP8266_DHCP_EN_CUR, ESP8266_CMD_SETUP, esp8266RxBuffer);// Send "AT+CWDHCP_CUR=s,a"
    else
        sendCommand(ESP8266_DHCP_EN_DEF, ESP8266_CMD_SETUP, esp8266RxBuffer); // Send "AT+CWDHCP_DEF=s,a"

    return readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
}

/* get list of IP and MAC addresses connected to SoftAP
 * only works when DHCP was enabled
 *
 * param out : return buffer to store received addresses
 * param len : length of buffer
 *
 * return OK > 0 else error
 */

int16_t ESP8266Class::SoftListIP(char *out, int len)
{
    unsigned int ret = 0;
    char *p;

    sendCommand(ESP8266_STATION_IP);    // Send AT+CWLIF\r\n

    /* Example Response:
     * AT+CWLIF\r\r\n
     * IP, MAC
     * IP, MAC
     * \r\nOK\r\n
     */

    ret = readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);

    if (ret > 0)
    {
        // must be more than AT+CWLIF\r\r\nOK\r\n
        if (ret > 17)
        {
            ret = 0;

            // skip AT+CWLIF\r\r\n
            p = strstr(esp8266RxBuffer, "IF");

            if (p != NULL)
            {
                p += 5;

                // copy up to OK or max length of buffer.
                while (*p != 'O' && ret < len-1) out[ret++] = *p++;
            }
         }
         else
            ret = 0;

        // terminate
        out[ret] = 0x0;
    }

    return ret;
}

/* get local IP (either CURRENT of DEFAULT)
 * param IP: return the IP
 * return : 1 = IP found else error or not found
 */
int16_t ESP8266Class::SoftGetLocalIP(bool mode, char * ip)
{
    char    *p;

    if (mode == CUR)
        sendCommand( ESP8266_SET_AP_IP_CUR, ESP8266_CMD_QUERY); // Send "AT+CIPAP_CUR?"
    else
        sendCommand( ESP8266_SET_AP_IP_CUR, ESP8266_CMD_QUERY); // Send "AT+CIPAP_DEF?"

    int16_t rsp = readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);

    if (rsp > 0)
    {
        // Look for "+CIPAP_CUR:ip:" or "+CIPAP_DEF:ip:"
        p = strstr(esp8266RxBuffer, "ip:");

        if (p != NULL)
        {
            p += 4;
            char * q = strchr(p, '"');
            if (q == NULL) return ESP8266_RSP_UNKNOWN;
            *q = 0x0;
            strncpy(ip, p, q - p + 1); // Copy string to temp char array:
            return 1;
        }
    }

    return rsp;
}


/* get local mac (either CURRENT of DEFAULT)
 * param MAC: return the MAC
 * return : 1 = MAC found else error or not found
 */
int16_t ESP8266Class::SoftGetLocalMAC(bool mode, char * mac)
{
    char    *p;

    if (mode == CUR)
        sendCommand( ESP8266_SET_AP_MAC_CUR, ESP8266_CMD_QUERY); // Send "AT+CIPAPMAC_CUR?"
    else
        sendCommand( ESP8266_SET_AP_MAC_DEF, ESP8266_CMD_QUERY); // Send "AT+CIPAPMAC_DEF?"

    int16_t rsp = readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);

    if (rsp > 0)
    {
        // Look for "+CIPAPMAC_CUR:" or "+CIPAPMAC_DEF:"
        p = strstr(esp8266RxBuffer, ":");

        if (p != NULL)
        {
            p += 2;
            char * q = strchr(p, '"');
            if (q == NULL) return ESP8266_RSP_UNKNOWN;
            *q = 0x0;
            strncpy(mac, p, q - p + 1); // Copy string to temp char array:
            return 1;
        }
    }

    return rsp;
}

/////////////////////
// TCP/IP Commands //
/////////////////////

/* make TCP connection
 *
 * return code
 * 1 new connection
 * 2 already connected
 * < 0 error
 */

int16_t ESP8266Class::tcpConnect(uint8_t linkID, const char * destination, uint16_t port, uint16_t keepAlive)
{
    clearBuffer();

    if (keepAlive > 0)
        // keepAlive is in units of 500 milliseconds.
        // Max is 7200 * 500 = 3600000 ms = 60 minutes.
        sprintf(esp8266RxBuffer,"%d,\"TCP\",\"%s\",%d,%d",linkID, destination, port, keepAlive/500);
    else
        sprintf(esp8266RxBuffer,"%d,\"TCP\",\"%s\",%d",linkID, destination, port);

    sendCommand(ESP8266_TCP_CONNECT, ESP8266_CMD_SETUP, esp8266RxBuffer);

    // Example good: CONNECT\r\n\r\nOK\r\n
    // Example bad:  DNS Fail\r\n\r\nERROR\r\n
    // Example meh:  ALREADY CONNECTED\r\n\r\nERROR\r\n
    int16_t rsp = readForResponses(RESPONSE_OK, RESPONSE_ERROR, CLIENT_CONNECT_TIMEOUT);

    if (rsp < 0)
    {
        // We may see "ERROR", but be "ALREADY CONNECTED".
        // Search for "ALREADY", and return success if we see it.
        char * p = searchBuffer("ALREADY");
        if (p != NULL)
            return 2;
        // Otherwise the connection failed. Return the error code:
        return rsp;
    }
    // Return 1 on successful (new) connection
    return 1;
}

/* send data over the TCP connections
 * linkID is the linkID used at connect
 * buf : contains data to sent
 * size: number of bytes in buffer  (max 2048)
 *
 * Will start sending as soon as number of bytes have been received
 *
 * return
 * OK : # of bytes send
 * ERROR : ESP8266_RSP_FAIL (-3)
 *
 */
int16_t ESP8266Class::tcpSend(uint8_t linkID, const uint8_t *buf, size_t size)
{
    char params[8] = {0};

    if (size > 2048)  return ESP8266_CMD_BAD;

    // the shield will "hang", if no byte is to be send
    if (size == 0) return 0;

    sprintf(params, "%d,%d", linkID, size);

    sendCommand(ESP8266_TCP_SEND, ESP8266_CMD_SETUP, params);

    int16_t rsp = readForResponses(RESPONSE_OK, RESPONSE_ERROR, COMMAND_RESPONSE_TIMEOUT);

    if (rsp != ESP8266_RSP_FAIL)
    {
        // print will use write to sent to ESP8266
        print((const char *)buf);

        rsp = readForResponse("SEND OK", COMMAND_RESPONSE_TIMEOUT);

        if (rsp > 0)
            return size;
    }

    return rsp;
}

/* read the buffer status and return in buf
 * LinkID : as used during connect
 * buf    : buffer to return the buffer status string
 * size   : length of buffer
 *
 * return example:
 * 1,20,15,10,200,7
 *
 * 1  :<link ID>         : ID of the connection (0~4), for multi-connect
 * 20 :<next segment ID> : next segment ID will be got by AT+CIPSENDBUF
 * 15 :<segment ID of which has sent> : the latest segment that
 *                                      sent(may not succeed);
 * 10 :<segment ID of which sent successfully>: the latest segment that
 *                                              sent successfully;
 * 200:<remain buffer size> :   TCP-send-buffer remain buffer size;
 * 7  :<queue number>        :  available TCP queue number, it’s not
 *                              reliable;when queue number is 0, no more
 *                              TCP data can be sent.
 *
 * return values
 * Ok   : # of bytes in buffer
 * Error:  <=0
 *  -5 buffer is not large enough
 *  -2 received data, but not OK. Data in buffer for debug
 */

int16_t ESP8266Class::tcpBufStatus(uint8_t linkID, char *buf, int size)
{
    // in case light version was requested to save RAM
#ifdef ESP_LIGHT
    return -1;
#else
    int i;

    sendCommand(ESP8266_TCP_BUFSTATUS, ESP8266_CMD_SETUP, (int) linkID);

    int16_t rsp = readForResponses(RESPONSE_OK, RESPONSE_ERROR, COMMAND_RESPONSE_TIMEOUT);

        // 1,20,15,10,200,7 \r\n
    if (rsp > 0)
    {
        char * p = esp8266RxBuffer;

        // copy to output buffer
        i=0;
        while(*p != '\r' && i < size) buf[i++]= *p++;

        // terminate
        if (i < size)
        {
            buf[i]= 0x0;
            return i;
        }
        else
        {   // buffer not large enough overwrite last byte
            buf[i-1]= 0x0;
            return (-5);
        }
    }

    else if (rsp == -2)
    {
            // we got something BUT NOT OK ... copy for debug
            for (i=0 ; i < bufferHead && i < size; i++) buf[i]=esp8266RxBuffer[i];

            // terminate
            if (i < size) buf[i] = 0x0;
            else
            {   // buffer not large enough
                buf[i-1] = 0x0;
                return (-2);
            }
    }

    return(rsp);
#endif
}

/* get server Timeout value
 */
int16_t ESP8266Class::tcpGetTimeout()
{
    char TimeO[8];

    sendCommand(ESP8266_SERVER_TIMEOUT, ESP8266_CMD_QUERY);  // Send AT+CIPSTO?\r\n

    // Look for the OK:
    int16_t rsp = readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);

    if (rsp > 0)
    {
        // Look for "+CIPSTTO" in the rxBuffer
        char *p = strstr(esp8266RxBuffer, ESP8266_SERVER_TIMEOUT);

        if (p != NULL)
        {
            p += strlen(ESP8266_SERVER_TIMEOUT) + 1;
            char * q = strchr(p, '\r');
            if (q == NULL) return ESP8266_RSP_UNKNOWN;

            strncpy(TimeO, p, q-p); // Copy string to temp char array:
            return(atoi(TimeO));
        }
    }

    return(rsp);
}

/* set timeout ESP8266 as TCP server, will disconnect to TCP client
 * that didn’t communicate with it even if timeout.
 */

int16_t ESP8266Class::tcpSetTimeout(uint16_t timeout)
{
    char params[2] = {0};
    sprintf(params, "%d", timeout);

    sendCommand(ESP8266_SERVER_TIMEOUT, ESP8266_CMD_SETUP, params);

    return readForResponses(RESPONSE_OK, RESPONSE_ERROR, COMMAND_RESPONSE_TIMEOUT);
}

/*
 * Check if specific segment sent successfully or not
 *
 * return :
 * OK : 1
 * Error : 0
 */

int16_t ESP8266Class::tcpSeqStatus(uint8_t linkID, uint8_t segID)
{
    // in case light version was requested to save RAM
#ifdef ESP_LIGHT
    return -1;
#else
    char params[4] = {0};
    sprintf(params, "%d,%d", linkID, segID);

    sendCommand(ESP8266_TCP_CHECKSEQ, ESP8266_CMD_SETUP, params);

    int16_t rsp = readForResponses(RESPONSE_OK, RESPONSE_ERROR, COMMAND_RESPONSE_TIMEOUT);

    // OK or ERROR
    if (rsp == ESP8266_RSP_FAIL ) return(0);

    return(1);
#endif
}

/*
 * Reset segment ID count
 *
 * If connection is not established or there are still TCP data wait for sending ERROR.
 *
 * return :
 * OK    : 1
 * Error : 0
 */

int16_t ESP8266Class::tcpResetSeg(uint8_t linkID, uint8_t segID)
{
// in case light version was requested to save RAM
#ifdef ESP_LIGHT
    return -1;
#else

    char params[4] = {0};
    int i;
    sprintf(params, "%d,%d", linkID, segID);

    sendCommand(ESP8266_TCP_RESETSEQ, ESP8266_CMD_SETUP, params);

    int16_t rsp = readForResponses(RESPONSE_OK, RESPONSE_ERROR, COMMAND_RESPONSE_TIMEOUT);

    // OK or ERROR
    if (rsp == ESP8266_RSP_FAIL ) return(0);

    return(1);
#endif
}

/* close TCP/UDP connection
 * linkID: was used during connect
 *
 * when ID=5, all connections will be closed
 * (ID=5 has no effect in server mode)
 *
 * Return
 * OK > 0
 *
 * ERROR :
 * ESP8266_RSP_FAIL : no such connection
 * ESP8266_RSP_UNKNOWN : error during communication
 * ESP8266_RSP_TIMEOUT : error during communication
 *
 */
int16_t ESP8266Class::close(uint8_t linkID)
{

    sendCommand(ESP8266_TCP_CLOSE, ESP8266_CMD_SETUP, (int) linkID);

    // Eh, client virtual function doesn't have a return value.
    // We'll wait for the OK or timeout anyway.
    return readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
}

/* set transfer mode
 *
 * ART-WiFi passthrough mode (transparent transmission) can only
 * be enabled in TCP single connection mode or UDP of which remote IP
 * and port won’t change (parameter <UDP mode> is 0 when using command
 * “AT+CIPSTART” to create a UDP transmission) .
 *
 * During UART-WiFi passthrough transmission, if it is TCP connection
 * and the TCP connection breaks, ESP8266 will keep trying to reconnect
 * until “+++” is inputed to quit from transmission.
 *
 * If it is a normal TCP transmission and TCP connection breaks,
 * ESP8266 will prompt “ [<link ID>,] CLOSED” , and won’t try to
 * reconnect. Users can call “AT+CIPSTART” to create a connection again
 * if it’s needed.
 */
int16_t ESP8266Class::setTransferMode(uint8_t mode)
{

    sendCommand(ESP8266_TRANSMISSION_MODE, ESP8266_CMD_SETUP, (mode > 0) ? 1 : 0);

    return readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
}

/* set to single or multiple connections
 * The different other routines expect multiple connection
 * this is called during begin()
 */
int16_t ESP8266Class::setMux(uint8_t mux)
{
    sendCommand(ESP8266_TCP_MULTIPLE, ESP8266_CMD_SETUP, (mux > 0) ? 1 : 0);

    return readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
}

/* configure as a TCP server
 *
 * port   : port to listen to
 * create : 1 is create, 0 is delete
 *
 * Return
 * OK > 0
 *
 * ERROR :
 * ESP8266_RSP_FAIL : no such connection
 * ESP8266_RSP_UNKNOWN : error during communication
 * ESP8266_RSP_TIMEOUT : error during communication
 *
 */

int16_t ESP8266Class::configureTCPServer(uint16_t port, uint8_t create)
{
    char params[8] = {0};
    if (create > 1) create = 1;

    sprintf(params, "%d,%d", create, port);

    sendCommand(ESP8266_SERVER_CONFIG, ESP8266_CMD_SETUP, params);

    return readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
}

/* stop TCP server that might be running
 * This has to be called at the BEGINNING of a sketch in case
 * a server is to be set with a different port than potentially
 * current.
 */
bool ESP8266Class::tcpStopServer()
{

    sendCommand(ESP8266_SERVER_CONFIG, ESP8266_CMD_SETUP, 0);

    if (readForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT) < 1)
        return false;

    if (reset() < 0)  return false;

    if (setMux(1)< 0) return false;

    return true;
}

int16_t ESP8266Class::ping(IPAddress ip)
{
    char ipStr[17];
    sprintf(ipStr, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    return ping(ipStr);
}

int16_t ESP8266Class::ping(char * server)
{
    char params[strlen(server) + 3];
    sprintf(params, "\"%s\"", server);
    // Send AT+Ping=<server>
    sendCommand(ESP8266_PING, ESP8266_CMD_SETUP, params);
    // Example responses:
    //  * Good response: +12\r\n\r\nOK\r\n
    //  * Timeout response: +timeout\r\n\r\nERROR\r\n
    //  * Error response (unreachable): ERROR\r\n\r\n
    int16_t rsp = readForResponses(RESPONSE_OK, RESPONSE_ERROR, COMMAND_PING_TIMEOUT);
    if (rsp > 0)
    {
        char * p = searchBuffer("+");
        p += 1; // Move p forward 1 space
        char * q = strchr(p, '\r'); // Find the first \r
        if (q == NULL)
            return ESP8266_RSP_UNKNOWN;
        char tempRsp[10];
        strncpy(tempRsp, p, q - p);
        return atoi(tempRsp);
    }
    else
    {
        if (searchBuffer("timeout") != NULL)
            return 0;
    }

    return rsp;
}

//////////////////////////
// Custom GPIO Commands //
//////////////////////////

/* these commands have been created by SparkFun and are not
 * part of the normal library. although the board shows and ADC
 * That can not be read.
 *
 * AT+PINMODE=<pin>,<mode>
 *
 * pin : 2,4,5,12,13,14,15
 *
 * mode :
 *  o: OUTPUT
 *  i: INPUT
 *  p: INPUT_PULLUP
 */
int16_t ESP8266Class::pinMode(uint8_t pin, uint8_t mode)
{
    char params[5];

    char modeC = 'i'; // Default mode to input
    if (mode == OUTPUT)
        modeC = 'o'; // o = OUTPUT
    else if (mode == INPUT_PULLUP)
        modeC = 'p'; // p = INPUT_PULLUP

    sprintf(params, "%d,%c", pin, modeC);
    sendCommand(ESP8266_PINMODE, ESP8266_CMD_SETUP, params);

    return readForResponses(RESPONSE_OK, RESPONSE_ERROR, COMMAND_RESPONSE_TIMEOUT);
}
/*
 * AT+PINWRITE=<pin>,<state>
 *
 * pin : 2,4,5,12,13,14,15
 *
 * state :     h: HIGH or  l: LOW
 */
int16_t ESP8266Class::digitalWrite(uint8_t pin, uint8_t state)
{
    char params[5]={0};

    char stateC = 'l'; // Default state to LOW

    if (state == HIGH)  stateC = 'h'; // h = HIGH

    sprintf(params, "%d,%c", pin, stateC);
    sendCommand(ESP8266_PINWRITE, ESP8266_CMD_SETUP, params);

    return readForResponses(RESPONSE_OK, RESPONSE_ERROR, COMMAND_RESPONSE_TIMEOUT);
}

/* AT+PINREAD=<pin>
 *
 * Response: 0 or 1 for LOW or HIGH.
 */

int8_t ESP8266Class::digitalRead(uint8_t pin)
{
    char params[3]={0};

    sprintf(params, "%d", pin);
    sendCommand(ESP8266_PINREAD, ESP8266_CMD_SETUP, params); // Send AT+PINREAD=n\r\n

    // Example response: 1\r\n\r\nOK\r\n
    if (readForResponses(RESPONSE_OK, RESPONSE_ERROR, COMMAND_RESPONSE_TIMEOUT) > 0)
    {
        if (strchr(esp8266RxBuffer, '0') != NULL)
            return LOW;
        else if (strchr(esp8266RxBuffer, '1') != NULL)
            return HIGH;
    }

    return -1;
}

//////////////////////////////
// Stream Virtual Functions //
//////////////////////////////

size_t ESP8266Class::write(uint8_t c)
{
    _serial->write(c);
}

int ESP8266Class::available()
{
    return _serial->available();
}

int ESP8266Class::read()
{
    return _serial->read();
}

int ESP8266Class::peek()
{
    return _serial->peek();
}

void ESP8266Class::flush()
{
    _serial->flush();       // softserial does not do anything
    rx_empty();             // read until empty
}

//////////////////////////////////////////////////
// Private, Low-Level, Ugly, Hardware Functions //
//////////////////////////////////////////////////

// change int param to char * params
void ESP8266Class::sendCommand(const char * cmd, enum esp8266_command_type type, int param)
{
    char params[2] = {0};

    sprintf(params, "%d", param);

    sendCommand(cmd, type, params);
}

void ESP8266Class::sendCommand(const char * cmd, enum esp8266_command_type type, const char * params)
{
    rx_empty();             // read from ESP8266 until empty

    // debug setting
    if (ESP_DEBUG > 0)
    {
        printf("Command to sent : AT%s",cmd);
        if (type == ESP8266_CMD_QUERY)      printf("?\n");
        else if (type == ESP8266_CMD_SETUP) printf("=%s\n", params);
        else printf("\n");
    }// end debug info */

    _serial->print("AT");
    _serial->print(cmd);

    if (type == ESP8266_CMD_QUERY)
        _serial->print('?');
    else if (type == ESP8266_CMD_SETUP)
    {
        _serial->print("=");
        _serial->print(params);
    }

    _serial->print("\r\n");
}

int16_t ESP8266Class::readForResponse(const char * rsp, unsigned int timeout)
{
    unsigned long timeIn = millis();    // Timestamp coming into function
    unsigned int received = 0;          // received keeps track of number of chars read

    clearBuffer();                      // Clear the class receive buffer (esp8266RxBuffer)

    while (timeIn + timeout > millis()) // While we haven't timed out
    {
        if (_serial->available())       // If data is available on UART RX
        {
            received += readByteToBuffer();
            if (searchBuffer(rsp))      // Search the buffer for goodRsp
            {
                return received;        // Return how many number of chars read
            }

            // paulvha : Sometimes response is READY! instead of OK
            if (rsp == RESPONSE_OK)
            {
                if (searchBuffer(RESPONSE_READY)) // Search the buffer for goodRsp
                {
                    return received;              // Return how number of chars read
                }
            }
        }
    }

    if (received > 0)                   // If we received any characters
        return ESP8266_RSP_UNKNOWN;     // Return unkown response error code
    else                                // If we haven't received any characters
        return ESP8266_RSP_TIMEOUT;     // Return the timeout error code
}

/* check for either GOOD (OK etc) and return number of bytes received
 * of check for FAIL (fail etc) and return ESP8266_RSP_FAIL.
 *
 * if bytes were received but neither response was received return ESP8266_RSP_UNKNOWN
 * if NO bytes received return ESP8266_RSP_TIMEOUT
 */
int16_t ESP8266Class::readForResponses(const char * pass, const char * fail, unsigned int timeout)
{
    unsigned long timeIn = millis();    // Timestamp coming into function
    unsigned int received = 0;          // received keeps track of number of chars read

    clearBuffer();                      // Clear the class receive buffer (esp8266RxBuffer)
    while (timeIn + timeout > millis()) // While we haven't timed out
    {
        if (_serial->available())       // If data is available on UART RX
        {
            received += readByteToBuffer();
            if (searchBuffer(pass))     // Search the buffer for goodRsp
                return received;        // Return how number of chars read

            // paulvha : Sometimes response is READY! instead of OK
            if (pass == RESPONSE_OK)
            {
                if (searchBuffer(RESPONSE_READY)) // Search the buffer for goodRsp
                    return received;              // Return how number of chars read
            }

            if (searchBuffer(fail))
                return ESP8266_RSP_FAIL;
        }
    }

    if (received > 0)               // If we received any characters
        return ESP8266_RSP_UNKNOWN; // Return unkown response error code
    else                            // If we haven't received any characters
        return ESP8266_RSP_TIMEOUT; // Return the timeout error code
}

//////////////////
// Buffer Stuff //
//////////////////
void ESP8266Class::clearBuffer()
{
    memset(esp8266RxBuffer, '\0', ESP8266_RX_BUFFER_LEN);
    bufferHead = 0;
}

unsigned int ESP8266Class::readByteToBuffer()
{
    // Read the data in
    char c = _serial->read();

    // Store the data in the buffer
    esp8266RxBuffer[bufferHead] = c;

    // For debug
    if (ESP_DEBUG > 1) printf("btb %c %x %d\n",c, (c & 0xff), bufferHead);

    //! TODO: Don't care if we overflow. Should we? Set a flag or something?
    // however the BUFFER is reset before we should NOT overrun
    bufferHead = (bufferHead + 1) % ESP8266_RX_BUFFER_LEN;

    return 1;
}

char * ESP8266Class::searchBuffer(const char * test)
{
    int bufferLen = strlen((const char *)esp8266RxBuffer);

    // If our buffer isn't full, just do an strstr
    if (bufferLen < ESP8266_RX_BUFFER_LEN)
        return strstr((const char *)esp8266RxBuffer, test);
}

ESP8266Class esp8266;
