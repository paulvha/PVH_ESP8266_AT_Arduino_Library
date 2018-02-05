/**********************************************************************
* VERSION 1.0 / PAULVHA / JANUARY 2018  / ESP8266-PVH-driver
*
* PVH_AT.h
*
* This an updated version for the ESP8266 AT WIFI shield driver
* for the Arduino. It contains a number of bugfixes, enhancements and
* new features that are not part of the original version (see below)
*
* Distributed as-is; no warranty is given.
***********************************************************************
* Original version :
*
ESP8266_AT.h
ESP8266 AT Command Definitions
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

//////////////////////
// Common Responses //
//////////////////////
const char RESPONSE_OK[] = "OK\r\n";
const char RESPONSE_ERROR[] = "ERROR\r\n";
const char RESPONSE_FAIL[] = "FAIL";
const char RESPONSE_READY[] = "READY!";


///////////////////////
// Basic AT Commands //
///////////////////////
const char ESP8266_TEST[] = "";              // Test AT startup
const char ESP8266_RESET[] = "+RST";         // Restart module
const char ESP8266_RESTORE[] = "+RESTORE";   // Factory reset
const char ESP8266_VERSION[] = "+GMR";       // View version info

const char ESP8266_ECHO_ENABLE[] = "E1";     // AT commands echo
const char ESP8266_ECHO_DISABLE[] = "E0";    // AT commands echo

const char ESP8266_SLEEP[] = "+SLEEP";       // Enter sleep mode
const char ESP8266_DEEP_SLEEP[] = "+GSLP";   // Enter deep-sleep mode

const char ESP8266_TXPOWER[] = "+RFPOWER";   // set radiofreqency TX power level directly
//!const char ESP8266_TXVDD[] = "+RFVDD";       // set radiofreqency TX power level based on VDD (TOUT can not be used)

const char ESP8266_UART_CUR[] = "+UART_CUR"; // UART configuration
const char ESP8266_UART_DEF[] = "+UART_DEF"; // UART configuration


////////////////////
// WiFi Functions //
////////////////////
const char ESP8266_WIFI_MODE_CUR[] = "+CWMODE_CUR";      // set/get CURRENT WiFi mode (sta/AP/sta+AP)
const char ESP8266_WIFI_MODE_DEF[] = "+CWMODE_DEF";      // set/get Default WiFi mode (sta/AP/sta+AP)

const char ESP8266_CONNECT_AP_CUR[] = "+CWJAP_CUR";      // connect to or get CURRENT AP
const char ESP8266_CONNECT_AP_DEF[] = "+CWJAP_DEF";      // connect to or get DEFAULT AP

const char ESP8266_LIST_AP[] = "+CWLAP";                 // List available AP's

const char ESP8266_DISCONNECT[] = "+CWQAP";              // Disconnect from AP

const char ESP8266_DHCP_EN_CUR[] = "+CWDHCP_CUR";        // Enable/disable current DHCP
const char ESP8266_DHCP_EN_DEF[] = "+CWDHCP_DEF";        // Enable/disable default DHCP
const char ESP8266_AUTO_CONNECT[] = "+CWAUTOCONN";       // Connect to AP automatically

const char ESP8266_SET_STA_IP_CUR[] = "+CIPSTA_CUR";     // Set CURRENT IP address of ESP8266 station
const char ESP8266_SET_STA_IP_DEF[] = "+CIPSTA_DEF";     // Set DEFAULT IP address of ESP8266 station

const char ESP8266_GET_STA_MAC_CUR[] = "+CIPSTAMAC_CUR"; // Get current MAC address of station
const char ESP8266_GET_STA_MAC_DEF[] = "+CIPSTAMAC_DEF"; // Get default MAC address of station

/////////////////////
// SOFTAP Commands //
/////////////////////
const char ESP8266_AP_CONFIG_CUR[] = "+CWSAP_CUR";     // Set softAP configuration CURRENT
const char ESP8266_AP_CONFIG_DEF[] = "+CWSAP_DEF";     // Set softAP configuration DEFAULT

const char ESP8266_SET_AP_MAC_CUR[] = "+CIPAPMAC_CUR"; // Set MAC address of current softAP
const char ESP8266_SET_AP_MAC_DEF[] = "+CIPAPMAC_DEF"; // Set MAC address of default softAP

const char ESP8266_SET_AP_IP_CUR[] = "+CIPAP_CUR";     // Set current IP address of ESP8266 softAP
const char ESP8266_SET_AP_IP_DEF[] = "+CIPAP_DEF";     // Set default IP address of ESP8266 softAP

const char ESP8266_STATION_IP[] = "+CWLIF";            // List station IP's connected to softAP

/////////////////////
// TCP/IP Commands //
/////////////////////
const char ESP8266_TCP_STATUS[] = "+CIPSTATUS";     // Get connection status
const char ESP8266_TCP_CONNECT[] = "+CIPSTART";     // Establish TCP connection or register UDP port
const char ESP8266_TCP_SEND[] = "+CIPSEND";         // Send Data
const char ESP8266_TCP_CLOSE[] = "+CIPCLOSE";       // Close TCP/UDP connection
const char ESP8266_GET_LOCAL_IP[] = "+CIFSR";       // Get local IP address
const char ESP8266_TCP_BUFSTATUS = "+CIPBUFSTATUS"; // returns the buffer status
const char ESP8266_TCP_CHECKSEQ = "+CIPCHECKSEQ";   // check where segment was sent succesfully
const char ESP8266_TCP_RESETSEQ = "+CIPBUFRESET";   // reset segment ID counter
const char ESP8266_TCP_MULTIPLE[] = "+CIPMUX";      // Set multiple connections mode
const char ESP8266_SERVER_CONFIG[] = "+CIPSERVER";  // Configure as server
const char ESP8266_TRANSMISSION_MODE[] = "+CIPMODE";// Set transmission mode
const char ESP8266_PING[] = "+PING";                // Function PING
const char ESP8266_SERVER_TIMEOUT[] = "+CIPSTO";    // Set timeout when ESP8266 runs as TCP server
//!const char ESP8266_SET_SaveTranslink[] = "+SAVETRANSLINK"; // Save transparent transmission link to Flash
//!const char ESP8266_FirmWareUpgrade[] = "+CIUPDATE"; // perform update of firmware of network
//!const char ESP8266_EnableIPD[] = "+CIPDINFO"; // Show remote IP and port with “+IPD”
//!const char ESP8266_ShowIPD[] = "+IPD"; // Receive network data

//////////////////////////
// Custom GPIO Commands //
//////////////////////////
const char ESP8266_PINMODE[] = "+PINMODE";          // Set GPIO mode (input/output)
const char ESP8266_PINWRITE[] = "+PINWRITE";        // Write GPIO (high/low)
const char ESP8266_PINREAD[] = "+PINREAD";          // Read GPIO digital value
