/************************************************************
 paulvha / version 1.0 / January 2018

 This sketch will set the Arduino  + ESP8266 AT shield as server.
 using port

 334 : talkback test: will display an received message and
       sent the input from the serial line (keyboard)

 For loopback, talkback and gpio test a seperate tclient.c is
 available in the folder "extra".  This has been tested between
 an Arduino and RaspberryPI-3  and also to Ubuntu 16.04 LTS

 A large number of bugfixes in the code and driver has been done
 to improve the stability as well as implementing new features
 and calls to driver. This sketch requires the ESP8266-PVH-driver
 to be installed.

  Development environment specifics:
  IDE: Arduino 1.8.6
  Hardware Platform: Arduino Uno
  ESP8266 WiFi Shield Version: 1.0

 Distributed as-is; no warranty is given.
**************************************************************
Based on The original version of this sketch :

ESP8266_Shield_Demo.h
SparkFun ESP8266 AT library - Demo
Jim Lindblom @ SparkFun Electronics
Original Creation Date: July 16, 2015
https://github.com/sparkfun/SparkFun_ESP8266_AT_Arduino_Library

This example demonstrates the basics of the SparkFun ESP8266
AT library. It'll show you how to connect to a WiFi network,
get an IP address, connect over TCP to as a server.


Development environment specifics:
  IDE: Arduino 1.6.5
  Hardware Platform: Arduino Uno
  ESP8266 WiFi Shield Version: 1.0

This code is released under the MIT license.


Distributed as-is; no warranty is given.
************************************************************/

//////////////////////
// Library Includes //
//////////////////////
// SoftwareSerial is required (even you don't intend on
// using it).
#include <SoftwareSerial.h>
#include <PVH_ESP8266WiFi.h>

//////////////////////////////
// WiFi Network Definitions //
//////////////////////////////
// Replace these two character strings with the name and
// password of your WiFi network.
const char mySSID[] = "yourSSIDhere";
const char myPSK[] = "yourPWDhere";

//////////////////////////////////////////////////
//           set new static IP if wanted        //
// Default :  no change wanted                  //
// Change  #define ipaddress ""                 //
// to something like #define "192.168.1.161";   //
//////////////////////////////////////////////////
#define ipaddress ""

char CLIENTIP[]= ipaddress;
char gateway[]="192.168.1.1";
char mask[]="255.255.255.0";

//////////////////////////////
// ESP8266Server definition //
//////////////////////////////
// (This is only global because it's called in both setup() and loop()).
////////////////////////////////////////////////
// port 334 will be used for loop-back test   //
////////////////////////////////////////////////
uint16_t _port = 334;
ESP8266Server server = ESP8266Server(_port);

// All functions called from setup() are defined below the
// loop() function. They modularized to make it easier to
// copy/paste into sketches of your own.
void setup()
{
  // Serial Monitor is used to control the demo and view debug information.
  Serial.begin(9600);
  serialTrigger(F("Press any key to begin."));

  //enable debug data from driver
  // (0 = disable (default), 1 = show commands, 2 = detailed)
  esp8266.debug(0);

  // initializeESP8266() verifies communication with the WiFi
  // shield, and sets it up.
  Serial.println(F("starting initializing"));
  initializeESP8266();

  // connectESP8266() connects to the defined WiFi network.
  Serial.println(F("start connecting"));
  connectESP8266();

  // displayConnectInfo prints the Shield's local IP/ MAC
  // and the network it's connected to.
  displayConnectInfo();

  // if a static address is needed, this can now be set
  setstatic();

  serialTrigger(F("Press any key to start talkback server."));

  serverSetup();
}

void loop()
{
  serverDemo();
}

void initializeESP8266()
{
  // esp8266.begin() verifies that the ESP8266 is operational
  // and sets it up for the rest of the sketch.
  // It returns either true or false -- indicating whether
  // communication was successul or not.
  // true

  int test = esp8266.begin();
  if (test != true)
  {
    Serial.println(F("Error talking to ESP8266."));
    errorLoop(test);
  }
  Serial.println(F("ESP8266 Shield Present"));

  // stop any server that might be running it has to be stopped
  // otherwise we can not set to listen on another port
  if (esp8266.tcpStopServer() == false)
  {
    Serial.println(F("Error during stopping server."));
    errorLoop(0);
  }

  // set speed to 19200
  // setSpeed(19200);

  // set to maximum antenne power half way. if you set this too high
  // the on-board antenna can not handle it well.
  esp8266.txpower(ESP8266_POWER_SET,40);

  // display firmware info
  firmware_info();
}

void connectESP8266()
{
  // The ESP8266 can be set to one of three modes:
  //  1 - ESP8266_MODE_STA   - Station only
  //  2 - ESP8266_MODE_AP    - Access point only
  //  3 - ESP8266_MODE_STAAP - Station/AP combo
  int retVal = setmode(ESP8266_MODE_STA);
  if (retVal < 0) errorLoop(retVal);
  Serial.println(F("Mode set to station"));

  // check that requested ssid is available
  retVal = checkssid(1);
  if (retVal <= 0) errorLoop(retVal);

  // There are 2 ways to connect: either do NOT check and just
  // disconnect in case we were connected:
  //esp8266.disconnect();

  // OR use esp8266.status() to indicate the ESP8266's WiFi connect status.
  // We might end up to be connected to the wrong network however
  // return values with option 1
  //     Success: 2, 3, 4, or 5 (ESP8266_STATUS_GOTIP, ESP8266_STATUS_CONNECTED, ESP8266_STATUS_DISCONNECTED, ESP8266_STATUS_NOWIFI)
  //    - Fail: <0 (esp8266_cmd_rsp)

  retVal= esp8266.status(1);
  if (retVal == ESP8266_STATUS_DISCONNECTED ||retVal == ESP8266_STATUS_NOWIFI )
  {
    // set to get IP adres from AP (in case set to static was set before)
    retVal = setDHCP();
    if (retVal < 0) errorLoop(retVal);

    Serial.print(F("Connecting to "));
    Serial.println(mySSID);

    // esp8266.connect([ssid], [psk]) connects the ESP8266
    // to a network.
    // On success the connect function returns a value >0
    // On fail, the function will either return:
    //  -1: TIMEOUT - The library has a set 30s timeout
    //  -3: FAIL - Couldn't connect to network.
    retVal = esp8266.connect(mySSID, myPSK);
    if (retVal < 0)
    {
      Serial.println(F("Error connecting"));
      errorLoop(retVal);
    }
  }
  else if (retVal < 0)
  {
      Serial.println(F("Error checking status"));
      errorLoop(retVal);
  }
}

/* initialize server and display IP to connected */
void serverSetup()
{

  // begin initializes a ESP8266Server object. It will
  // start a server on the port specified in the object's
  // constructor (in global area)

  server.begin();
  Serial.print(F("Server started! Go to "));
  Serial.print(esp8266.localIP());
  Serial.print(F("  port : "));
  Serial.println(_port);
  Serial.println();
}

/* wait for client to connect */
void serverDemo()
{
  // available() is an ESP8266Server function which will
  // return an ESP8266Client object for printing and reading.
  // available() has one parameter -- a timeout value. This
  // is the number of milliseconds the function waits,
  // checking for a connection.
  ESP8266Client client = server.available(500);

  if (client)
  {
    Serial.println(F("Client detected"));

    talkback(client);

    // close the connection:
    client.stop();
    Serial.println(F("Client disconnected"));
  }

}

 /* talk back test */
 void talkback( ESP8266Client client)
 {
    Serial.println(F("Talk back client connected!"));
    int i = 0, j = 0 ;
    char keyb[20];
    char input[40];

    // flush any pending input
    client.flush();

    // check that client is still connected
    while (client.connected())
    {
      // read data and skip header
      // retry 2 times
      i = readData(client, input, sizeof(input), 2);

      // if any data, display and sent back
      if (i > 0)
      {
        input[i] = 0x0;                // terminate
        Serial.print(F("FROM: "));     // to screen
        Serial.print(input);           // to screen
      }

      // check for keyboard input
      if (Serial.available())
      {
         keyb[j] = Serial.read();

        // if enter, sent it
        if (keyb[j++] == '\n')
        {
            keyb[j] = 0x0;              // terminate
            Serial.print(F("TO  : "));  // to screen
            Serial.print(keyb);         // to screen
            client.print(keyb);         // to remote
            j = 0;
        }

      } // available
    } // while
}

//////////////////////////////////////////////////////////
/////////////// supporting routines //////////////////////
//////////////////////////////////////////////////////////
/* errorLoop prints an error code, then loops forever.*/
void errorLoop(int error)
{
  Serial.print(F("Error: ")); Serial.println(error);
  Serial.println(F("Looping forever."));

  // reset the board (maximizing chance to react
  // again when sketch is restarted)
  esp8266.reset();

  for (;;);
}

/* serialTrigger prints a message, then waits for something
 * to come in from the serial port.
 */
void serialTrigger(String message)
{
  Serial.println();
  Serial.println(message);
  Serial.println();

  while (!Serial.available());

  while (Serial.available())
    Serial.read();

}

/*  set new UART and interface speed once you have done this,
 *  the ESP8266 will keep this speed. You have to reset before
 *  exiting the program, or unplug the power from the board.
 *
 *  SoftSerial will have most of the time difficulties when
 *  setting speed about 56K
 */

void setSpeed(unsigned long speed)
{
  if (esp8266.setBaud(speed) == false)
  {
    Serial.println(F("Error setting UART speed."));
    errorLoop(0);
  }

  // time to settle
  delay(500);

  // set new speed in interface
  int test = esp8266.begin(speed);
  if (test != true)
  {
    Serial.println(F("Error setting speed."));
    errorLoop(test);
  }

  Serial.print(F("ESP8266 Set to "));
  Serial.println(speed);
}

/* display connected information */
void displayConnectInfo()
{
  char buf[24];
  memset(buf, 0, sizeof(buf));

  // esp8266.localIP returns an IPAddress variable with the
  // ESP8266's current local IP address.
  IPAddress myIP = esp8266.localIP();
  Serial.print(F("My IP : ")); Serial.println(myIP);

  // get MAC
  int retVal = esp8266.localMAC(buf);
  if (retVal < 0)
  {
    Serial.print(F("Error during reading MAC"));
    errorLoop(retVal);
  }
  Serial.print(F("My MAC: ")); Serial.println(buf);

  // esp8266.getAP() can be used to check which AP the
  // ESP8266 is connected to. It returns an error code.
  // The connected AP is returned by reference as a parameter.
  memset(buf, 0, sizeof(buf));

  retVal = esp8266.getAP(buf);

  if (retVal > 0)
  {
    Serial.print(F("Connected to: "));
    Serial.println(buf);
  }
  else
  {
    Serial.print(F("Not Connected "));
    errorLoop(retVal);
  }
}

/* will set DHCP mode to it will get it's IP from the AP (opposite to Static)
 * station 0 = Soft-AP DHCP               ESP8266_DHCP_SOFTAP
 *         1 = Station DHCP               ESP8266_DHCP_STATION
 *         2 = soft-AP and Station DHCP   ESP8266_DHCP_BOTH
 *
 * action  0 = disable                    ESP8266_DISABLE
 *         1 = enable                     ESP8266_ENABLE
 *
 * return
 * OK > 0
 * Error =< 0
 */
int16_t setDHCP()
{
  return esp8266.SetDHCP(CUR, ESP8266_DHCP_STATION ,ESP8266_ENABLE);
}


/* will set a static address once you have called this it will
 * keep your static address if you reset the Arduino and do NOT call static
 * This can be reset using SetDHCP before your next connect() or
 * perform a complete power-off of the board.
 *
 */
void setstatic()
{
   // check that new IP is requested
  if (strlen(CLIENTIP) == 0) return;

  if (esp8266.setlocalIP(CUR, CLIENTIP, gateway, mask) > 0)
  {
    Serial.print(F("My new static IP: "));
    Serial.println(esp8266.localIP());
  }
  else
  {
    Serial.print(F("Error during setting static"));
    errorLoop(0);
  }
}

/* display Firmware version information */
void firmware_info()
{
  char ATversion[35];
  char SDKversion[8];
  char compileTime[25];

  memset(ATversion,0,sizeof(ATversion));
  memset(SDKversion,0,sizeof(SDKversion));
  memset(compileTime,0,sizeof(compileTime));

  if (esp8266.getVersion(ATversion,SDKversion, compileTime) > 0)
  {
    Serial.print(F("ATversion   : "));
    Serial.println(ATversion);
    Serial.print(F("SDKversion  : "));
    Serial.println(SDKversion);
    Serial.print(F("CompileTime : "));
    Serial.println(compileTime);
  }
}

/* detect whether requested SSID is available
 * disp = 1 : will display received information otherwhise
 * disp = 0 : will only detect
 *
 * output is raw format : (4,"homelan",-45,"c0:c1:c0:58:ca:e3",1,0)
 * 4  : WPA_WPA2_PSK
 *                       0 OPEN
 *                       1 WEP
 *                       2 WPA_PSK
 *                       3 WPA2_PSK
 *                       4 WPA_WPA2_PSK
 *
 * homelan             :  SSID of AP
 * -45                 :  signal strength
 * "c0:c1:c0:58:ca:e3" :  MAC address
 * 1                   :  channel
 * 0                   :  <freq offset> frequency offset of AP,unit:KHz.
 *                         <freq offset> / 2.4 to get unit ppm”
 *
 * return
 * OK >= 0, else error < 0
 */
int checkssid(int disp)
{
  char  buf[64];
  memset(buf,0,sizeof(buf));

  if (disp)
  {
    Serial.print(F("Try to detect "));
    Serial.println(mySSID);
  }

  int retVal = esp8266.detectAP(mySSID, buf, sizeof(buf), 10000);

  if (retVal > 0)
  {
    if (disp) Serial.println(buf);
  }
  else
  {
    Serial.println(F(" not detected"));
  }

  return(retVal);
}

/*  The ESP8266 can be set to one of three modes:
 *  1 - ESP8266_MODE_STA   - Station only
 *  2 - ESP8266_MODE_AP    - Access point only
 *  3 - ESP8266_MODE_STAAP - Station/AP combo
 *
 *  return:
 *  OK  >= 0
 *  Error < 0
 */
int setmode(int mode)
{
  // Use esp8266.getMode() to check which mode it's in:
  int retVal = esp8266.getMode();

  // If it's not in the requested mode.
  if (retVal != mode)
  {
    retVal = esp8266.setMode(mode);

    if (retVal < 0)
    {
      Serial.println(F("Error setting mode."));
    }
  }

  return retVal;
}

/* read data from remote and strip +IPD.... header
 * client = client socket
 * out = buffer to store output
 * len = length of buffer
 * retry = retry count for checking
 *
 * if len is zero, The read data after the header
 * will be displayed
 *
 * return:
 * number of bytes received and stored in optional buffer
 * version 2.0 : optimized read loop
 */
int readData(ESP8266Client client, char *out, int len, int retry)
{
   int wait = 0;
   char c;
   int i = 0, ch_count = 0;
   char ch_len[5];
   int retry_loop = retry;

   out[0] = 0x0;                       // terminate return message in case of empty message received

  // Try at least loop times before returning
  while (retry_loop > 0)
  {
     retry_loop--;
     delay(100);                      // wait 100ms to handle any delay between characters

    // available() will return the number of characters
    // currently in the receive buffer.
    while (client.available() )
    {
      // get character from buffer
      c = client.read();

      // parse +IPD ....  header. format is : +IPD,0,4:paul
      if (wait < 3)
      {
        if (wait == 2) ch_len[i++] = c;  // after the second comma : store character count digit(s)

        if (c == ',') wait++;            // count comma's

        else if (c == ':')               // end of header
        {
          ch_len[i] = 0x0;               // terminate buffer
          ch_count= atoi(ch_len);        // calculate length of message
          wait = 3;                      // indicate header has been parsed
          i = 0;                         // reset storage counter
        }

        continue;                        // skip rest of loop
      }

      // if no buffer length, just display character on serial/screen
      if (len == 0) Serial.write(c);
      else
      {
        //output to provided buffer
        out[i++] = c;
        if (i == len) return len;
      }

      // if we got atleast ONE character, but not all increase retry_loop to keep trying to get all
      // we should not expect/hope this loopcount will be reached. This is only to prevent a dead-lock
      if (--ch_count > 0) retry_loop = 10;

      // seems we got them all, no need for waiting and retry
      else return i;
    }
  }

  // return count of characters in buffer
  return i;
}
