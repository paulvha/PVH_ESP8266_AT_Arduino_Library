/************************************************************
 paulvha / version 1.0 / January 2018

 This example demonstrates the basics of the SparkFun ESP8266
 AT library. It'll show you how to set an softAP, option to change
 it's address and/or it's MAC, as well as different timeout and show
 who is connected to the AP. A client can connect with a loopback test
 using a port (default 333)

 For loopback, talkback and gpio test a seperate tclient.c is
 available in the folder "extra".  This has been tested between
 an Arduino and RaspberryPI-3  and also to Ubuntu 16.04 LTS

 Development environment specifics:
  IDE: Arduino 1.8.6
  Hardware Platform: Arduino Uno
  ESP8266 WiFi Shield Version: 1.0

 A large number of bugfixes in the code and driver has been done
 to improve the stability as well as implementing new features
 and calls to driver. This sketch requires the PVH-ESP8266 driver
 to be installed.

 Distributed as-is; no warranty is given.
**************************************************************
Based on the original version of this sketch :

ESP8266_Shield_Demo.h
SparkFun ESP8266 AT library - Demo
Jim Lindblom @ SparkFun Electronics
Original Creation Date: July 16, 2015
https://github.com/sparkfun/SparkFun_ESP8266_AT_Arduino_Library


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
// SoftwareSerial is required (even you don't intend on using it).
#include <SoftwareSerial.h>
#include <PVH_ESP8266WiFi.h>

//////////////////////////////
// ESP8266Server definition //
//////////////////////////////
// (This is only global because it's called in both setup() and loop()).

char serverSSID[]="testserver";      // name
char serverPSK[]="sparkfun";         // password
uint16_t server_port = 333;

ESP8266Server server = ESP8266Server(server_port);

//////////////////////////////////////////////////////
//   set new newSoftIP[]= "" if no change wanted    //
//////////////////////////////////////////////////////
char newSoftIP[] = "";
//char newSoftIP[]="192.168.2.100";   // new IP address.
char newSoftGateway[] ="192.168.2.1"; // new Gateway
char newSoftMask[]="255.255.255.0";   // new Mask

//////////////////////////////////////////////////////
//                                                  //
// SOFT MAC SHOULD BE UNIQUE AND NOT EQUAL TO ANY   //
// OTHER MAC ON THE NETWORK, INCLUDING THE HARDWARE //
// MAC OF THE ESP8266                               //
//                                                  //
// set new newSoftMAC[]= "" if no change wanted     //
//////////////////////////////////////////////////////
char newSoftMAC[] = "";
//char newSoftMAC[]="5e:cf:7f:e3:7F:11";   // new MAC address.

// Defined globally to save RAM bytes
// buffer used in many routines
char buf[40];

// All functions called from setup() are defined below the
// loop() function. They modularized to make it easier to
// copy/paste into sketches of your own.
void setup()
{
  // Serial Monitor is used to control the demo and view debug information.
  Serial.begin(9600);
  serialTrigger(F("Press any key to begin."));

  // enable debug data from driver
  // (0 = disable (default), 1 = show commands, 2 = detailed)
  esp8266.debug(0);

  // initializeESP8266() verifies communication with the WiFi
  // shield, and sets it up.
  Serial.println(F("starting initializing"));
  initializeESP8266();

  // create soft Access point.
  Serial.println(F("Setting Soft Access Point"));
  initSoftAP();

  serialTrigger(F("Press any key to start loopback SoftAP server."));

  soft_server_setup();
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


  // stop any server that might be running
  // it has to be stopped otherwise we can not set to listen on another port
  // This will reset the ESP8266 and must be called before obtaining IP-address
  // it will cause a delay of a couple of seconds for the board to reset
  Serial.println(F("Stop any server running + board reset ( 2 - 3 seconds)"));
  if (esp8266.tcpStopServer() == false)
  {
    Serial.println(F("Error during stopping server."));
    errorLoop(0);
  }

  // set to maximum antenne power half way. if you set this too high
  // the on-board antenna can not handle it well.
  esp8266.txpower(ESP8266_POWER_SET,40);

  // set speed between ESP8266 and Arduino to 19200
  // setSpeed(19200);

  // display firmware info
  firmware_info();
}

/* creates a soft Access pooint and will display the
 * default IP, and will set  new requested softIP and
 * softMac addresses, if requested
 */

void initSoftAP()
{
  // The ESP8266 can be set to one of three modes:
  //  1 - ESP8266_MODE_STA   - Station only
  //  2 - ESP8266_MODE_AP    - Access point only
  //  3 - ESP8266_MODE_STAAP - Station/AP combo
  int16_t retVal = setmode(ESP8266_MODE_AP);
  if (retVal < 0) errorLoop(retVal);
  Serial.println(F("Mode set to SoftAP and station"));

  // create soft access point
  if (esp8266.softAP(serverSSID, serverPSK) <= 0)
  {
    Serial.println(F("setting softAP "));
    errorLoop(0);
  }

  // display MAC
  if (esp8266.SoftGetLocalMAC(CUR,buf))
  {
    Serial.print(F("local MAC "));
    Serial.println(buf);
  }
  else
    Serial.println(F("Error reading MAC"));

  // display IP
  if (esp8266.SoftGetLocalIP(CUR,buf))
  {
    Serial.print(F("local IP "));
    Serial.println(buf);
  }
  else
   Serial.println(F("Error reading IP"));

  // option to change softAP IP
  setsoftIP();

  // option to change softAP MAC
  setsoftMAC();
}

/* initialize server and display IP to connected */
void soft_server_setup()
{
  // begin initializes a ESP8266Server object. It will
  // start a server on the port specified in the object's
  // constructor (in global area)

  server.begin();

  // set timeout 60 seconds
  setSoftTimeout(60);

  // display IP
  if (! esp8266.SoftGetLocalIP(CUR,buf))
  {
    Serial.println(F("Error reading IP"));
    errorLoop(0);
  }

  Serial.print(F("Connect to address : "));
  Serial.print(buf);
  Serial.print(F(" port for GPIO test : "));
  Serial.println(server_port);
  Serial.println();
}

/* wait for client to connect */
void serverDemo()
{
  // must define as static otherwise reset to 1 every loop.
  static unsigned int check_connect = 1;

  // available() is an ESP8266Server function which will
  // return an ESP8266Client object for printing and reading.
  // available() has one parameter -- a timeout value. This
  // is the number of milliseconds the function waits,
  // checking for a connection.
  ESP8266Client client = server.available(500);

  if (client)
  {
    Serial.println(F("Client detected"));

    loopback(client);

    // close the connection:
    client.stop();
    Serial.println(F("Client disconnected"));
  }

  // check who is connected
  if (--check_connect == 0)
  {
    // display who is connected (if any)
    softwhoconnected();

    // every 35 loops (about every 10 seconds)
    check_connect = 35;
  }


}

/* loop backtest */
void loopback( ESP8266Client client)
{
    Serial.println(F("Loop back client connected!"));
    int i = 0;

    // flush any pending input
    client.flush();

    // check that client is still connected
    while (client.connected())
    {
      // read data and skip header
      i = readData(client, buf, sizeof(buf),5);

      // if any data, display and sent back
      if (i > 0)
      {
        buf[i] = 0x0;                  // terminate
        Serial.print(F("Received : "));// to screen
        Serial.println(buf);           // to screen
        client.print(buf);             // to remote
      }
    }
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

/* set softserver timeout value */
void setSoftTimeout(uint16_t timeout)
{
  // set timeout
  if (esp8266.tcpSetTimeout(timeout) > 0)
  {
      // display timeout
      int16_t retVal = esp8266.tcpGetTimeout();
      if (retVal > 0)
      {
        Serial.print(F("Server timeout "));
        Serial.println(retVal);
      }
      else
        Serial.println(F("Error reading Timeout"));
  }
  else
   Serial.println(F("Error setting  Timeout"));
}


/* display who is connected to the softAP (if any) */
void softwhoconnected()
{
  int retval = esp8266.SoftListIP(buf, sizeof(buf));

  if (retval < 0)
  {
    Serial.println(F("Error during reading who is connected."));
    errorLoop(retval);
  }

  // only display if there are connected
  if (retval > 0)
  {
    Serial.println(F("Connected to Access Point:\n    IP     ,    MAC"));
    Serial.println(buf);
  }
}

/*
 *  will set a new softAP IP address
 */
void setsoftIP()
{
  // check that new IP is requested
  if (strlen(newSoftIP) == 0) return;

  if (esp8266.SoftSetLocalIP(CUR, newSoftIP, newSoftGateway, newSoftMask) > 0)
  {
      // display IP
    if (esp8266.SoftGetLocalIP(CUR,buf))
    {
      Serial.print(F("New local IP "));
      Serial.println(buf);
    }
    else
      Serial.println(F("Error reading new IP"));
  }
  else
  {
    Serial.print(F("Error during setting softAP IP"));
    errorLoop(0);
  }
}

/*
 *  will set a new softAP MAC
 */
void setsoftMAC()
{
  // check that new IP is requested
  if (strlen(newSoftMAC) == 0) return;

  if (esp8266.SoftSetLocalMAC(CUR, newSoftMAC) > 0)
  {
      // display IP
    if (esp8266.SoftGetLocalMAC(CUR,buf))
    {
      Serial.print(F("New local MAC "));
      Serial.println(buf);
    }
    else
      Serial.println(F("Error reading new MAC"));
  }
  else
  {
    Serial.print(F("Error during setting softAP MAC"));
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
 * Version 2.0 : optimized for better results
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

        // prevent buffer overrun
        if (i == len-1)
        {
          out[i] = 0x0;  // terminate
          return i;
        }
      }

      // if we got atleast ONE character, but not all increase retry_loop to keep trying to get all
      // we should not expect/hope this loopcount will be reached. This is only to prevent a dead-lock
      if (--ch_count > 0) retry_loop = 10;

      // seems we got them all, no need for waiting and retry
      else return i;
    }
  }

  out[i] = 0x0;
  // return count of characters in buffer
  return i;
}
