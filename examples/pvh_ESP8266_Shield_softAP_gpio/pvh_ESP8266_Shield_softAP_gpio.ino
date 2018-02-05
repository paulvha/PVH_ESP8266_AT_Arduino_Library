/************************************************************
 paulvha / version 1.0 / January 2018

 This sketch will set the Arduino  + ESP8266 AT shield as a SoftAP server
 with it's own SSDI and password and performs gpio test.

 For loopback,talkback and gpio test a seperate tclient.c is
 available in the folder "extra".  This has been tested between
 an Arduino and RaspberryPI-3 (Jessie version) and also to Ubuntu 16.04 LTS

 This example demonstrates the basics of the SparkFun ESP8266
 AT library. It'll show you how to set an SoftP server, handle
 requests to set digital pin HIGH and LOW, have digital output
 blink at different speeds and read digital input from both the
 Arduino and Sparkfun ESP8266 AT board. Reading an analog port is
 also supported but only from the Arduino. The ESP8266 firmware does
 not support reading the ADC.  (default port 330)

 A large number of bugfixes in the code and driver has been done
 to improve the stability as well as implementing new features
 and calls to driver. This sketch requires the PVH-ESP8266 driver
 to be installed.

 Development environment specifics:
  IDE: Arduino 1.8.6
  Hardware Platform: Arduino Uno
  ESP8266 WiFi Shield Version: 1.0

 Distributed as-is; no warranty is given.
**************************************************************
parts of the code is based on this sketch :

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
// CHANGE BELOW TO SET YOUR OWN SSID AND PASSWORD
char serverSSID[]="testserver";      // name
char serverPSK[]="sparkfun";         // password
uint16_t server_port = 330;

ESP8266Server server = ESP8266Server(server_port);

//////////////////////////
// GPIO definitions     //
//////////////////////////

// CHANGE TO YOUR OWN PINS
#define A_LEDS     4         // max leds on Arduino (must equal the number A_LED entries)
#define ESP_LEDS   2         // max leds on ESP (must equal the number ESP_LED entries)

#define TOTLEDS A_LEDS + ESP_LEDS

uint8_t A_LED[A_LEDS] = {11,12,0,0};  // REPLACE 0 with the pin number for the led on the Arduino
uint8_t ESP_LED[ESP_LEDS] = {14,12};  // REPLACE 0 with the pin number for the led on the ESP8266

#define A_BUTS     2         // max Button on Arduino (must equal the number A_BUT entries)
#define ESP_BUTS   2         // max Button on ESP (must equal the number ESP_BUT entries)

uint8_t A_BUT[A_BUTS] = {7,0};        // REPLACE 0 with the pin number for the button on the Arduino
uint8_t ESP_BUT[ESP_BUTS] = {13,0};   // REPLACE 0 with the pin number for the button on the ESP8266

#define MAX_ADC   5         // maximum ADC ports starting at 0

// structure to capture the led status
struct gpio_led{
  char          board;   // either A (Arduino) or E (ESP8266)
  uint8_t       gpio;    // pin number
  int           c_state; // current state HIGH or LOW
  int           r_state; // request state
  int           rate;    // number of blinks per minute
  unsigned long next;    // when is next blink in milli seconds
} gpio_led;

struct gpio_led LEDS[TOTLEDS];

// Defined globally to save RAM bytes
// buffer used in many routines
char *p, buf[40];

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
  Serial.println(F("initializing ESP8266"));
  initializeESP8266();

  // create soft Access point.
  Serial.println(F("initializing Access Point"));
  initSoftAP();

  // initialize GPIO's (must be after setting the board)
  Serial.println(F("initializing GPIO"));
  initGPIO();

  serialTrigger(F("Press any key to start server."));

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
}

void initSoftAP()
{
  // The ESP8266 can be set to one of three modes:
  //  1 - ESP8266_MODE_STA   - Station only
  //  2 - ESP8266_MODE_AP    - Access point only
  //  3 - ESP8266_MODE_STAAP - Station/AP combo

  // set default to station
  //int16_t retVal = setmode(DEF,ESP8266_MODE_STA);
  //if (retVal < 0) errorLoop(retVal);

  uint16_t retVal = setmode(CUR,ESP8266_MODE_AP);
  if (retVal < 0) errorLoop(retVal);
  Serial.println(F("Mode set to SoftAP and station"));

  // create soft access point
  if (esp8266.softAP(serverSSID, serverPSK) <= 0)
  {
    Serial.println(F("setting softAP "));
    errorLoop(0);
  }
}

/* initialize the GPIO
 * this must be called after the board is initialized
 */
void initGPIO()
{
  int i;

  // set LEDs as output
  for (i=0 ; i< A_LEDS; i ++)
  {
    if (A_LED[i] != 0x0) {pinMode(A_LED[i],OUTPUT); digitalWrite(A_LED[i], LOW);}
  }

  for (i=0 ; i< ESP_LEDS; i ++)
  {
    if (ESP_LED[i] != 0x0) {esp8266.pinMode(ESP_LED[i],OUTPUT); esp8266.digitalWrite(ESP_LED[i], LOW);}
  }

  // input button
  for (i=0 ; i< A_BUTS; i ++)
  {
    if (A_BUT[i] != 0x0) pinMode(A_BUT[i],INPUT_PULLUP);
  }

  for (i=0 ; i< ESP_BUTS; i ++)
  {
    if (ESP_BUT[i] != 0x0) esp8266.pinMode(ESP_BUT[i],INPUT_PULLUP);
  }

  // reset LEDs structure
  for(i=0 ; i < TOTLEDS; i++)  LEDS[i].gpio = 0xff;
}

/* initialize server and display IP to connected */
void soft_server_setup()
{
  // begin initializes a ESP8266Server object. It will
  // start a server on the port specified in the object's
  // constructor (in global area)

  server.begin();

  // set client timeout 3 minutes
  setSoftTimeout(300);

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
  // available() is an ESP8266Server function which will
  // return an ESP8266Client object for printing and reading.
  // available() has one parameter -- a timeout value. This
  // is the number of milliseconds the function waits,
  // checking for a connection.
  ESP8266Client client = server.available(500);

  if (client)
  {
    Serial.println(F("Client detected"));

    handleGPIO(client);

    // close the connection:
    client.stop();
    Serial.println(F("Client disconnected"));
  }
}

/* handle GPIO request */
void handleGPIO(ESP8266Client client)
{
    Serial.println(F("GPIO handling client connected!"));
    int i = 0, ret;
    char command[8];

    // flush any pending input
    client.flush();

    //init LedActions
    ClrLedAction();

    // check that client is still connected
    while (client.connected())
    {

      // read data and skip header (retry 5 times)
      i = readData(client, buf, sizeof(buf), 5);

      // if any data, check for command
      if (i > 0)
      {
            Serial.print(F("Received : "));// to screen
            Serial.println(buf);           // to screen

            p = strstr(buf, "SWITCH");
            if (p != NULL)
            {
              ret = SWITCH();

              if (ret < 0)  strcpy(buf,"FAIL");
              else strcpy(buf,"OK");
            }

            p = strstr(buf, "BLINK");
            if (p != NULL)
            {
              ret = BLINK();
              if (ret < 0)  strcpy(buf,"FAIL");
              else sprintf(buf,"OK");
            }

            p = strstr(buf, "STATUS");
            if (p != NULL)
            {
              ret = STATUS();
              if (ret < 0)  strcpy(buf,"FAIL");

              if (ret == LOW)  strcpy(buf,"LOW");
              else strcpy(buf,"HIGH");
            }

            p = strstr(buf, "ADC");
            if (p != NULL)
            {
              ret = GetAdc();
              if (ret < 0)  strcpy(buf,"FAIL");
              else sprintf(buf,"value:%d", ret);
            }

            Serial.print(F("Sending feedback : "));// to screen
            Serial.println(buf);           // to screen
            client.print(buf);             // to remote
      }

    // update led's (if necessary)
    DoLedAction();

    }
}

// clear LED's and actions
void ClrLedAction()
{
  int i;

  for(i=0 ; i < TOTLEDS; i++)
  {
    // set output low
    if (LEDS[i].gpio != 0xff)
    {
       LEDS[i].r_state = LOW;
       SetLedPin(i);
    }

   // set as available
   LEDS[i].gpio = 0xff;
  }
}

/* set led output pin
 * ind : index in LEDS table
 */
void SetLedPin(int ind)
{
  // set pin to requested state
  if (LEDS[ind].board == 'A') digitalWrite(A_LED[LEDS[ind].gpio], LEDS[ind].r_state);
  else esp8266.digitalWrite(ESP_LED[LEDS[ind].gpio],LEDS[ind].r_state);

  // set current state to requested state
  LEDS[ind].c_state = LEDS[ind].r_state;
}

// perform requested action(s) on LED
void DoLedAction()
{
  int i;
  unsigned long t;

  for(i=0; i < TOTLEDS; i++)
  {
    // if action for GPIO is set
    if (LEDS[i].gpio != 0xff)
    {

      // if blink action was set
      if (LEDS[i].rate > 0)
      {
        t= millis();

        // has blink timeout expired
        if (t >= LEDS[i].next)
        {
          // switch output level
          if (LEDS[i].c_state == HIGH) LEDS[i].r_state = LOW;
          else LEDS[i].r_state = HIGH;

          // set next timeout : 1 minute is 60 x 1000 ms
          LEDS[i].next = t + (unsigned long) 60000 / LEDS[i].rate;
        }
      }

      // if current state does not equal requested state change LED output
      if  (LEDS[i].c_state != LEDS[i].r_state) SetLedPin(i);

     }
  }
}

/* Look for entry with current pin
 * else look for empty entry
 * returns table entry number or -1 if not able to
 *
 */
int FindLedEntry(char board, int pin)
{
  int ind;

  // look for existing entry for this pin & board
  ind = 0;
  while (ind < TOTLEDS)
  {
    if (LEDS[ind].gpio == pin)
    {
      if (LEDS[ind].board == board) break;
    }
    ind++;
  }

  // if NOT found
  if (ind == TOTLEDS)
  {
      // look for empty entry
      ind = 0;
      while (ind < TOTLEDS)
      {
        if (LEDS[ind].gpio == 0xff) break;
        ind++;
      }

      if (ind == TOTLEDS)
      {
          Serial.println(F("not enough table entries"));
          return -1;
      }
  }

  return(ind);
}

/* Detect whether LED request is for Arduino or ESP8266
 * obtain the pinnumber and check it is valid
 * get table entry for that pin (current or new)
 * set pin and board in table entry
 *
 * return table entry else -1
 */

int FindBoardLed()
{
  int     pin, ind;
  char    board;

  // check for led on Arduino
  p = strstr(buf, "A_LED");

  // if found
  if (p != NULL)
  {
    // skip A_LED to get pin
    p += 5;

    // 1 = offset 0
    pin = *p - 0x31;

    // check within limits and set (valid)
    if (pin > A_LEDS  || pin < 0 || A_LED[pin] == 0x0) return -1;

    // Arduino board
    board = 'A';
  }
  else
  {
    // check for led on ESP8266
     p = strstr(buf, "ESP_LED");

     // if NOT found
     if (p == NULL)
     {
      Serial.print(F("invalid request "));
      Serial.println(buf);
      return -1;
     }

     // skip ESP_LED to get pin
     p += 7;

     // LED1 is offset 0
     pin = *p - 0x31;

     // check within limit and set
     if (pin > ESP_LEDS  || pin < 0 || ESP_LED[pin] == 0x0) return -1;

     board = 'E';
  }

   // get table entry
  ind = FindLedEntry(board, pin);

  // if no entry available (should NEVER happen... but does software know that ?)
  if (ind == -1) return -1;

  LEDS[ind].board = board;    // save board
  LEDS[ind].gpio = pin;       // save GPIO

  return(ind);
}


/* Set a digital pin HIGH or LOW on the Arduino or the SparkfunESP8266 AT
 *
 *format example SWITCH(A_LED1,ON) / SWITCH(A_LED1,OFF) / SWITCH(ESP_LED1,ON) / SWITCH(A_LED2,ON)
 *
 */
int SWITCH()
{
  int     ind;

   // parse command, set board, pin and LEDS-table entry
  ind = FindBoardLed();

  // make sure it was valid
  if (ind == -1) return -1;

  LEDS[ind].rate = 0;         // no blink

  // skip and ,O
  p += 3;

  // set high when ON else low
  if (*p == 'N') LEDS[ind].r_state = HIGH;
  else LEDS[ind].r_state = LOW;

  return 0;
}

/*
 * Set a digital pin HIGH or LOW at a specified rate on the Arduino or the
 * Sparkfun ESP8266 AT
 * format example BLINK(A_LED1,10) / BLINK(A_LED2,20) / BLINK(ESP_LED1,30)
 * The 10, 20 OR 30 in the examples above is change frequency per MINUTE
 */
int BLINK()
{
   char   freq[5];      // capture blink frequency per minute
   int    ind, i;

   // parse command, set board, led and table entry
  ind = FindBoardLed();

  // check it was valid
  if (ind == -1) return -1;

  // skip and #,
  p+= 2;

  // get blink frequency
  while(*p != ')' && i < 5) freq[i++] = *p++;

  // terminate
  freq[i]=0x0;

  // set blink rate
  LEDS[ind].rate = atoi(freq);

  // set to start now
  LEDS[ind].next = 0;

  return 0;
}

/* Get the current status of a digital pin on either the Arduino or he
 * Sparkfun ESP8266 AT board
 *
 *format example  STATUS(A_BUT1)  STATUS(ESP_BUT1)
 *
 */
int STATUS()
{
  int     pin;

  // check for led on Arduino
  p = strstr(buf, "A_BUT");

  // if found
  if (p != NULL)
  {
    // skip A_BUT to get pin
    p += 5;

    // 1 = offset 0
    pin = *p - 0x31;

    // check within limits and set (valid)
    if (pin > A_BUTS  || pin < 0 || A_BUT[pin] == 0x0) return -1;

    return digitalRead(A_BUT[pin]);
  }
  else
  {
    // check for ESP8266
     p = strstr(buf, "ESP_BUT");

     // if NOT found
     if (p == NULL)
     {
      Serial.print(F("invalid request "));
      Serial.println(buf);
      return -1;
     }

     // skip ESP_BUT to get pin
     p += 7;

     // 1 is offset 0
     pin = *p - 0x31;

     // check within limit and set (valid)
     if (pin > ESP_BUTS  || pin < 0 || ESP_BUT[pin] == 0x0) return -1;

     return esp8266.digitalRead(ESP_BUT[pin]);
  }
}

/* Get the value of an analog input
 * format :ADC(A_x) : x= port ( e.g. 0)
 *
 * While there is an ADC on the ESP8266, there is no option implemented in the firmware to read
 * that ADC value. Hence this will only check for the Arduino
 */
int GetAdc()
{
  int     pin;

  // check for led on Arduino
  p = strstr(buf, "(A_");

  // if found
  if (p != NULL)
  {
    // skip A_ to get pin
    p += 3;

    pin = *p - 0x30;

    // check within limits and set
    if (pin > MAX_ADC  || pin < 0) return -1;

    return analogRead(pin);
  }

  return -1;
}

//////////////////////////////////////////////////////////
// supporting routines ///////////////////////////////////
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

/*  The ESP8266 can be set to one of three modes:
 *  1 - ESP8266_MODE_STA   - Station only
 *  2 - ESP8266_MODE_AP    - Access point only
 *  3 - ESP8266_MODE_STAAP - Station/AP combo
 *
 *  conf is either CUR or DEF
 *
 *  return:
 *  OK  >= 0
 *  Error < 0
 */
int setmode(bool conf, int mode)
{
  // Use esp8266.getMode() to check which mode it's in:
  int retVal = esp8266.getMode(conf);

  // If it's not in the requested mode.
  if (retVal != mode)
  {
    retVal = esp8266.setMode(conf,mode);

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
 *
 */
int readData(ESP8266Client client, char *out, int len, int retry)
{
   bool wait = 1;
   char c;
   int i = 0;

  // Try at least loop times
  while (retry > 0)
  {
     retry--;
     delay(10);    // wait 10ms

    // available() will return the number of characters
    // currently in the receive buffer.
    while (client.available() )
    {
      // get character from buffer
      c = client.read();

      // skip +IPD ....  header
      if (wait)
      {
        if (c == ':')  wait = 0;
        continue;
      }

      if (len == 0) Serial.write(c);
      else
      {
        //output to provided buffer
        out[i++] = c;
        out[i] = 0x0; // terminate
        if (i == len) return len;
      }
    }
  }

  // return count in buffer
  return i;
}
