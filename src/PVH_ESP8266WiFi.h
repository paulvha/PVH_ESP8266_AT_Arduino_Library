/**********************************************************************
* VERSION 1.0 / PAULVHA / JANUARY 2018  / ESP8266-PVH-driver
*
* PVH_ESP8266WiFi.h
*
* This an updated version for the ESP8266 AT WIFI shield driver
* for the Arduino. It contains a number of bugfixes, enhancements and
* new features that are not part of the original version (see below)
*
* Distributed as-is; no warranty is given.
***********************************************************************
* Original version :
*
SparkFunESP8266WiFi.h
ESP8266 WiFi Shield Library Main Header File
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
***********************************************************************/

#ifndef _SPARKFUNESP8266_H_
#define _SPARKFUNESP8266_H_

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <IPAddress.h>
#include "PVH_ESP8266Client.h"
#include "PVH_ESP8266Server.h"


/*********************************************************************
 * set for light version (in case the bytes are needed in the sketch)
 *
 * This will save RAM space (~120 bytes) and program storage (~350 bytes)
 *
 * The following impact:
 *
 * Reduced buffer
 *      ESP8266_RX_BUFFER_LEN 80 instead of 200 (because of removed functions)
 *
 * Functions that will return -1:
 *      detectAP(char *ssid, char *out, int len, unsigned long timeout);
 *      setBaud(bool mode, unsigned long baud)
 *      getVersion(char *ATversion, char *SDKversion, char *compileTime)
 *      sleep(int mode)
 *      AutoConnect(int action)
 *      listAP(char *out, int len, unsigned long timeout )
 *      setlocalMAC(bool mode, char * mac)
 *      SoftSetLocalMAC(bool mode, char * mac)
 *      SoftSetLocalIP(bool mode, char * IP, char *gateway, char *mask)
 *      tcpBufStatus(uint8_t linkID, char *buf, int size)
 *      tcpSeqStatus(uint8_t linkID, uint8_t segID)
 *      tcpResetSeg(uint8_t linkID, uint8_t segID)
 *
 * Remove comments before #define to set light version
 */

//#define ESP_LIGHT 1

/////////////////////
// Pin Definitions //
/////////////////////
#define ESP8266_SW_RX   9   // ESP8266 UART0 RXI goes to Arduino pin 9
#define ESP8266_SW_TX   8   // ESP8266 UART0 TXO goes to Arduino pin 8

///////////////////////////////
// Command Response Timeouts //
///////////////////////////////
#define COMMAND_RESPONSE_TIMEOUT 5000
#define COMMAND_PING_TIMEOUT     3000
#define WIFI_CONNECT_TIMEOUT     50000
#define COMMAND_RESET_TIMEOUT    5000
#define CLIENT_CONNECT_TIMEOUT   5000

#define ESP8266_MAX_SOCK_NUM     5      // max 5 clients
#define ESP8266_SOCK_NOT_AVAIL   255

// TX power settings
#define ESP8266_POWER_SET 0        // set DB directly
#define ESP8266_POWER_MIN 0
#define ESP8266_POWER_MAX 82

#define ESP8266_POWER_VDD33     1  // set depending on VDD33 (TOUT must be suspended)
#define ESP8266_POWER_VDD33_MIN 0
#define ESP8266_POWER_VDD33_MAX 1024

static SoftwareSerial swSerial(ESP8266_SW_TX, ESP8266_SW_RX);

// DHCP setting
#define ESP8266_DHCP_SOFTAP  0
#define ESP8266_DHCP_STATION 1
#define ESP8266_DHCP_BOTH    2

// enable/disable
#define ESP8266_DISABLE 0
#define ESP8266_ENABLE  1

// Current and default configuration
#define CUR  0
#define DEF 1

typedef enum esp8266_cmd_rsp {
    ESP8266_CMD_BAD = -5,
    ESP8266_RSP_MEMORY_ERR = -4,
    ESP8266_RSP_FAIL = -3,
    ESP8266_RSP_UNKNOWN = -2,
    ESP8266_RSP_TIMEOUT = -1,
    ESP8266_RSP_SUCCESS = 0
};

typedef enum esp8266_wifi_mode {
    ESP8266_MODE_STA = 1,
    ESP8266_MODE_AP = 2,
    ESP8266_MODE_STAAP = 3
};

typedef enum esp8266_command_type {
    ESP8266_CMD_QUERY,
    ESP8266_CMD_SETUP,
    ESP8266_CMD_EXECUTE
};

typedef enum esp8266_sleep {
    ESP_SLEEP_DISABLE = 0,
    ESP_SLEEP_LIGHT = 1,
    ESP_SLEEP_MODEM = 2
};

typedef enum esp8266_encryption {
    ESP8266_ECN_OPEN,
    ESP8266_ECN_WPA_PSK,
    ESP8266_ECN_WPA2_PSK,
    ESP8266_ECN_WPA_WPA2_PSK
};

typedef enum esp8266_connect_status {
    ESP8266_STATUS_GOTIP = 2,
    ESP8266_STATUS_CONNECTED = 3,
    ESP8266_STATUS_DISCONNECTED = 4,
    ESP8266_STATUS_NOWIFI = 5
};

typedef enum esp8266_serial_port {
    ESP8266_SOFTWARE_SERIAL,
    ESP8266_HARDWARE_SERIAL
};

typedef enum esp8266_socket_state {
    AVAILABLE = 0,
    TAKEN = 1,
};

typedef enum esp8266_connection_type {
    ESP8266_TCP,
    ESP8266_UDP,
    ESP8266_TYPE_UNDEFINED
};

typedef enum esp8266_tetype {
    ESP8266_CLIENT,
    ESP8266_SERVER
};

struct esp8266_ipstatus
{
    uint8_t linkID;
    esp8266_connection_type type;
    IPAddress remoteIP;
    uint16_t port;
    esp8266_tetype tetype;
};

struct esp8266_status
{
    esp8266_connect_status stat;
    esp8266_ipstatus ipstatus[ESP8266_MAX_SOCK_NUM];
};

class ESP8266Class : public Stream
{
public:
    ESP8266Class();

    bool begin(unsigned long baudRate = 9600, esp8266_serial_port serialPort = ESP8266_SOFTWARE_SERIAL);

    ///////////////////////
    // Basic AT Commands //
    ///////////////////////
    bool test();
    int reset();
    int restore();
    bool echo(bool enable);

    bool debug(int act);    // enable of disable debug (0 = disable, 1 = show commands, 2 = detailed)
    void rx_empty(void);

    int16_t getVersion(char * ATversion, char * SDKversion, char * compileTime);
    int16_t sleep(int mode);
    int16_t Deepsleep(uint32_t SleepTime);

    bool setBaud(unsigned long baud);
    bool setBaud(bool mode, unsigned long baud);

    int16_t getVDD_txpower();
    int16_t txpower(int mode, int level);

    ////////////////////
    // WiFi Functions //
    ////////////////////
    int16_t getMode();
    int16_t getMode(bool stat);

    int16_t setMode(esp8266_wifi_mode mode);
    int16_t setMode(bool stat, esp8266_wifi_mode mode);

    int16_t connect(const char * ssid);
    int16_t connect(const char * ssid, const char * pwd);
    int16_t connect(bool mode, const char * ssid, const char * pwd);

    int16_t getAP(char * ssid);
    int16_t getAP(bool mode, char * ssid);

    int16_t localMAC(char * mac);
    int16_t localMAC(bool mode, char * mac);

    IPAddress localIP();
    int16_t disconnect();

    int16_t setlocalIP(bool mode, char * IP, char *gateway, char *mask);
    int16_t setlocalMAC(bool mode, char * mac);

    int16_t GetStatusDHCP(bool mode);
    int16_t SetDHCP(bool mode, int station, int action);

    int16_t AutoConnect(int action);

    int listAP(char *out, int len, unsigned long timeout );
    int detectAP(char *ssid, char *out, int len, unsigned long timeout);

    /////////////////////
    // SOFTAP Commands //
    /////////////////////
    int16_t softAP(const char * ssid, const char * pwd);
    int16_t SoftConfigureAP(bool mode, const char * ssid, const char * pwd, int channel, int encoding);

    int16_t SoftListIP(char *out, int len);

    int16_t SoftSetLocalMAC(bool mode, char * mac);
    int16_t SoftGetLocalMAC(bool mode, char * mac);

    int16_t SoftSetLocalIP(bool mode, char * IP, char *gateway, char *mask);
    int16_t SoftGetLocalIP(bool mode, char * ip);

    /////////////////////
    // TCP/IP Commands //
    /////////////////////
    int16_t status(bool status_new = 0);
    int16_t updateStatus();
    bool    tcpStopServer();
    int16_t tcpConnect(uint8_t linkID, const char * destination, uint16_t port, uint16_t keepAlive);
    int16_t tcpSend(uint8_t linkID, const uint8_t *buf, size_t size);

    int16_t tcpBufStatus(uint8_t linkID, char *buf, int size);
    int16_t tcpSeqStatus(uint8_t linkID, uint8_t segID);
    int16_t tcpResetSeg(uint8_t linkID, uint8_t segID);

    int16_t tcpGetTimeout();
    int16_t tcpSetTimeout(uint16_t timeout);

    int16_t close(uint8_t linkID);
    int16_t setTransferMode(uint8_t mode);
    int16_t setMux(uint8_t mux);
    int16_t configureTCPServer(uint16_t port, uint8_t create = 1);
    int16_t ping(IPAddress ip);
    int16_t ping(char * server);

    //////////////////////////
    // Custom GPIO Commands //
    //////////////////////////
    int16_t pinMode(uint8_t pin, uint8_t mode);
    int16_t digitalWrite(uint8_t pin, uint8_t state);
    int8_t digitalRead(uint8_t pin);

    ///////////////////////////////////
    // Virtual Functions from Stream //
    ///////////////////////////////////
    size_t write(uint8_t);
    int available();
    int read();
    int peek();
    void flush();

    friend class ESP8266Client;
    friend class ESP8266Server;

protected:
    Stream* _serial;
    unsigned long _baud;

private:
    //////////////////////////
    // Command Send/Receive //
    //////////////////////////
    void sendCommand(const char * cmd, enum esp8266_command_type type, int param);
    void sendCommand(const char * cmd, enum esp8266_command_type type = ESP8266_CMD_EXECUTE, const char * params = NULL);
    int16_t readForResponse(const char * rsp, unsigned int timeout);
    int16_t readForResponses(const char * pass, const char * fail, unsigned int timeout);

    //////////////////
    // Buffer Stuff //
    //////////////////
    /// clearBuffer() - Reset buffer pointer, set all values to 0
    void clearBuffer();

    /// readByteToBuffer() - Read first byte from UART receive buffer
    /// and store it in rxBuffer.
    unsigned int readByteToBuffer();

    /// searchBuffer([test]) - Search buffer for string [test]
    /// Success: Returns pointer to beginning of string
    /// Fail: returns NULL
    //! TODO: Fix this function so it searches circularly
    char * searchBuffer(const char * test);

    esp8266_status _status;

    uint8_t sync();
};

extern ESP8266Class esp8266;

#endif
