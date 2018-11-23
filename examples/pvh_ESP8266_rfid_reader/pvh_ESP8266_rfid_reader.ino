/************************************************************
ESP8266_RFID_Reader   

Version draft : November 2018 paulvha
        proof of concept version 1.0 

//////////////// short description //////////////////////////////////

This sketch is demonstrating how to connect a Sparkfun WIFI shield and a Sparkfun nano-M6E simultaneous RFID shield to an MEGA2560. 

The WIFI connection will be setup as server on the local network, listening to port 80 (defined by WIFI_PORT)

The EPC of the tags will be constant read and are stored in an array, only ONE entry for each unique tag.

By connecting to the server, one can sent from a remote location the commands 
get_cnt : this will return the number of unique EPC's stored on the Mega2560
get_epc : will return the EPC, comma seperated in case of more than one
clr_epc : will clear the EPC's stored on the Mega2560

A demo version for this remote connection is available at epconnect.c in the extra directory. 
This has been tested on a raspberry Pi and Ubuntu 18.04

///////////////// Wiring /////////////////////////////////////////////////
WIFI SHIELD:

The Sparkfun WIFI shield is placed right on top of the MEGA2560. (*0)
The on-board switch is set to HW UART (*1)

THe RX-line from the UART connection is connected to pin 18 (RX1) on the MEGA2560 ( IMPORTANT *2)
THe TX-line from the UART connection is connected to pin 19 (TX1) on the MEGA2560 ( IMPORTANT *2)

The RESET pin from the ESP8266 GPIO connector, is connected to pin 11 (*3) on the MEGA2560

Notes:
*0 : Make sure to leave room for the antenna and put isolation tape over the USB connector to prevent
     short circuit with the WIFI UART pins.
     
*1 : the swserial can NOT be used on the MEGA2560 for 2 reasons. 
     A. The pin 8, standard WIFI shield software TX line does NOT have a interrupt associated in the ATMEGA-chip
     B. SoftSerial can only handle ONE interrupt RX at the time. The NANO shield will be used for that.

*2 : As the switch is on HW, the TX and RX signals are not only connected to the UART, but 
     also to pin 1 and 2 of the shield. This is already used by the MEGA2560 for the USB connection. 
     You MUST make sure that these pin 1 and 2 are NOT CONNECTED to the MEGA2560 (just cut them off)
     
     You can decide to use UART port 2 or 3. Change that in the variable WIFI_SERIAL_TYPE below and connect to 
     the right GPIO on the MEGA

*3 : hard reset in optional. By default this is set in this sketch but can be disabled by setting
     WIFI_HARD_RESET_PIN to 0 (zero). However using hard-reset will increase the chance to a good start of the
     wifi-connection


NANO-M6E SIMULTANEOUS RFID READER:

In this example the Nano is connected with 4 wires to the MEGA2560 (so !! NOT !! placed on top of the Arduino/ESP8266 shield)

The on-board switch is set to HW UART (*4) 

From the Serial UART connection
GND  connect to a GND on the WIFI-SHIELD / MEGA2560
5V   connect to a GND on the WIFI-SHIELD / MEGA2560  (*5)
RXI  connect to pin 13 on the WIFI-SHIELD / MEGA2560 (*6)
TXI  connect to pin 12 on the WIFI-SHIELD / MEGA2560 (*6)

The Buzzer, pin 8 and 9 on the shield, is not used in this sketch.

Notes :
*4 : The standard softserial connection (pin 2 and 3) can not be used as it does NOT have a interrupt associated in the ATMEGA-chip.
     As the standard SPARKFUN library, for now, only supports softserial we sticked to that. Later version might make use of another UART serial
     port of the MEGA2560

*5 : Be aware that the shield needs enough power. This setup should be able to handle up to 5BM. If you need more use additional power
     supply e.g. a battery or external source instead of 5V from MEGA2560. In that case you can decide to NOT connect the 5V to the MEGA2560
     to prevent damage. you MUST keep the GND connection !

*6 : These pin can be changed with RFID_SoftSerial_TX and RFID_SoftSerial_RX below. However be aware that only the following pins on the MEGA could be
     used for RFID_SoftSerial_RX : Pins: 10, 11, 12, 13,  50, 51, 52, 53,  62, 63, 64, 65, 66, 67, 68, 69, as they have an interrupt associated with them.
     
///////////////// Development environment specifics: //////////////////////////////
  
  Hardware Platform: Arduino MEGA2560
  Sparkfun     ESP8266 WiFi Shield Version: 1.0 ( https://www.sparkfun.com/products/13287)
  Sparkfun     Nano ME6 simultaneous reader (https://www.sparkfun.com/products/14066)

  IDE: Arduino 1.8.6.
  Sparkfun library : (https://github.com/sparkfun/SparkFun_Simultaneous_RFID_Tag_Reader_Library)
  WIFI library ; PVH_ESP8266WiFi (https://github.com/paulvha/PVH_ESP8266_AT_Arduino_Library)

*****************************************************************************************************
This code is released under the MIT license.

Distributed as-is; no warranty is given. No support... nothing... apart from whishing you good-luck !
******************************************************************************************************/

   //////////////////////
   // Library Includes //
   //////////////////////

#include <SoftwareSerial.h>             // SoftwareSerial is required (even you don't intend on using it).
#include <PVH_ESP8266WiFi.h>            // adjusted library for the WIFI shield (use the latest !)
#include "SparkFun_UHF_RFID_Reader.h"   // Library for controlling the M6E Nano module

  /////////////////////////////////////////////////
  //            WiFi Network Definitions        //
  ////////////////////////////////////////////////
  
  /* MUST: define the serial connection for the WIFI shield
   * On shield switch to SW ; RX = pin 9, TX = pin 8, (unless this is a mega, than it is pin 10)
   * On shield switch to HW : TX and RX pin on the UART connection
   * Valid options are :
     ESP8266_SOFTWARE_SERIAL
     ESP8266_HARDWARE_SERIAL
     ESP8266_HARDWARE_SERIAL1
     ESP8266_HARDWARE_SERIAL2
     ESP8266_HARDWARE_SERIAL3 */
#define WIFI_SERIAL_TYPE ESP8266_HARDWARE_SERIAL1

  /* MUST: define the serial speed to communicate with the shield. Different versions of the
   * software may use different speeds */
#define WIFI_SERIAL_SPEED 9600

 /* OPTIONAL : by setting a pin number other than 0, a hard reset is performed on the ESP8266  
  * as part of initialization below. This can help to have a better start  */
#define WIFI_HARD_RESET_PIN 11

  /* MUST: Replace these two character strings with the name & password of your WiFi network.*/
const char mySSID[] = "yourSSIDhere";
const char myPSK[] = "yourPWDhere";

  /* MUST: define the TXpower between 0 and 82.  In case of onboard antenna 40 is a good level,
   * but can be increased to experiment or if you use an external antenna */
#define WIFI_TX_POWER 40

  /* MUST: ESP8266Server port definition */
#define WIFI_PORT 80

  /* Optional : set WIFI driver debug
   * (0 = disable (default), 1 = show commands, 2 = detailed) */
#define WIFI_DEBUG 0

  /* MUST: will disconnect to TCP client that didn’t communicate within timeout (seconds)*/
#define SERVER_TIMEOUT 20

  /* MUST: define maximum data-length before sending. The data can be send in chunks
   * this can be set depending on local memory or remote buffer  */
#define OUTMAX 64

   /* OPTIONAL: response header for content to client in case of HTML 
    * This has been moved within the sub-routine add-output. We can  
    * apply the F() macro only within a subroutine to save memory space  */
    
  /////////////////////////////////////////////////
  //            RFID shield Definitions          //
  /////////////////////////////////////////////////

  /* MUST: As we communication softserial, you must define the pins to use 
   * see header of this sketch */
#define RFID_SoftSerial_TX 13
#define RFID_SoftSerial_RX 12
  
  /* MUST: define the serial speed to communicate with the shield 
   * DO NOT SET THIS HIGHER WHEN USING SOFTSERIAL */
#define RFID_SERIAL_SPEED 38400

 /* MUST: Define the region in the world you are in
  * It defines the frequency to use that is available in your part of the world
  * 
  * Valid options are :
  * REGION_INDIA, REGION_JAPAN, REGION_CHINA, REGION_EUROPE, REGION_KOREA,
  * REGION_AUSTRALIA,  REGION_NEWZEALAND, REGION_NORTHAMERICA
  */
#define RFID_Region REGION_EUROPE

  /* MUST: define the Readpower between 0 and 27dBm. if only using the
   * USB power, do not go above 5dbm (= 500) */
#define RFID_Read_Power 500

  /////////////////////////////////////////////////
  //           Program Definitions               //
  /////////////////////////////////////////////////

/* MUST: define the maximum table size for discovered / unique EPC */
#define EPC_COUNT 50        // number of unique EPC's
#define EPC_ENTRY 12        // max EPC bytes 

/* OPTIONAL : display debug level/information */
#define PRMDEBUG 0

  
  ///////////////////////////////////////////////////////////
  //            NO changes needed after this point        //
  //////////////////////////////////////////////////////////
  //                   GLOBAL VARIABLES                   //
  //////////////////////////////////////////////////////////
  
//Create instances
SoftwareSerial softSerial(RFID_SoftSerial_RX, RFID_SoftSerial_TX); //RX, TX
RFID nano;   
ESP8266Server server = ESP8266Server(WIFI_PORT);

// general variable for tmp count/return values
int val1; 

// create array to hold the discovered EPC's
uint8_t EPC_recv[EPC_COUNT][EPC_ENTRY];

// string to hold body message to reply
String reply;

  ////////////////////////////////////
  //            Setup program       //
  ////////////////////////////////////

/* initialize the hardware and software*/
void setup() 
{
  /* Serial Monitor is used to control view debug information.*/
  Serial.begin(115200);
  serialTrigger(F("Press any key to begin."));
  
  ////////////////////////////////////////////////////////
  ///           setup network connection                //
  ////////////////////////////////////////////////////////

  /* enable debug data from WIFI driver
   * (0 = disable (default), 1 = show commands, 2 = detailed) */
  esp8266.debug(WIFI_DEBUG);

  /* Verifies  & setup communication with the WiFi shield.*/
  initializeESP8266();

  /* connect to the defined WiFi network.*/
  connectESP8266();

  ////////////////////////////////////////////////////////
  ///           setup RFID connection                   //
  ////////////////////////////////////////////////////////

  initialize_RFID();

  // initialize array
  init_array();
  Serial.println(F("Array initialized"));
  
  ////////////////////////////////////////////////////////
  ///           Start program                           //
  ////////////////////////////////////////////////////////
 
  serialTrigger(F("Press any key to start network and reading"));
  serverSetup();
}


  ////////////////////////////////////
  //            Loop program       //
  ////////////////////////////////////
void loop() 
{
  Check_EPC();         // store unique EPC (if any) into array
  Server_comms();      // handle remote requests
}

  ////////////////////////////////////////////////////////
  ///           ESP8266 routines                        //
  ////////////////////////////////////////////////////////

/* Verifies  & setup communication with the WiFi shield.*/
void initializeESP8266()
{
  /* Enable hard reset. Driver can decide to use in case the shield does not respond
   * or the user can reset before start */
  if (WIFI_HARD_RESET_PIN > 0)
  {
    Serial.println(F("perform hard reset on Wifi-shield + 2 sec wait"));
    esp8266.enableResetPin(WIFI_HARD_RESET_PIN);
    esp8266.hard_reset();
  }
  
  /* Setup the serial communication port and speed, verify that the ESP8266 communicationis operational.
   * It returns either true or false -- indicating whether communication was successul or not. */
  val1 = esp8266.begin(WIFI_SERIAL_SPEED, WIFI_SERIAL_TYPE);
  if (val1 != true)
  {
    Serial.println(F("Error talking to ESP8266."));
    errorLoop(val1);
  }
  Serial.println(F("ESP8266 Shield Present"));

  /* stop any server that might be running. It has to be stopped otherwise we can not set to listen 
   * on another port. This will reset the ESP8266 and must be called befor obtaining IP-address */
  if (esp8266.tcpStopServer() == false)
  {
    Serial.println(F("Error during stopping server."));
    errorLoop(0);
  }

  /* set to transmit power. if you set this too high the on-board antenna can not handle it well.*/
  esp8266.txpower(ESP8266_POWER_SET, WIFI_TX_POWER);
}

/* set the wifi shield in the right mode and connect to the WIFI network */
void connectESP8266()
{
  /* The ESP8266 can be set to one of three modes:
   *  1 - ESP8266_MODE_STA - Station only
   *  2 - ESP8266_MODE_AP - Access point only
   *  3 - ESP8266_MODE_STAAP - Station/AP combo
   */
  val1 = esp8266.getMode();
  if (val1 != ESP8266_MODE_STA)
  { 
    // If it's not in station mode.
    val1 = esp8266.setMode(ESP8266_MODE_STA);

    if (val1 < 0)
    {
      Serial.println(F("Error setting mode."));
      errorLoop(val1);
    }
  }
  else if (val1 < 0)
  {
      Serial.println(F("Error getting mode."));
      errorLoop(val1);
  }
 
  Serial.println(F("Mode set to station"));

  /* Indicates the ESP8266's WiFi connect status.
   * A return value :
   *     1 indicates the device is already connected. 
   *     0 indicates disconnected. 
   *     Negative values equate to communication errors. */
  val1 = esp8266.status();
  if (val1 < 0)
  {
      Serial.println(F("Error getting status."));
      errorLoop(val1);
  }
  else if (val1 == 0)
  {
    Serial.print(F("Connecting to "));
    Serial.println(mySSID);

    /* esp8266.connect([ssid], [psk]) connects the ESP8266 to a network.
     * return value 
     *   0 (ESP8266_RSP_SUCCESS)
     *  -1: TIMEOUT - The library has a set 30s timeout
     *  -3: FAIL - Couldn't connect to network. */
    val1 = esp8266.connect(mySSID, myPSK);

    if (val1 < 0)
    {
      Serial.println(F("Error connecting"));
      errorLoop(val1);
    }
  }
}

/* begin initializes a ESP8266Server object. It will start a server on the port 
 * specified in the object's constructor (in global area) */
void serverSetup()
{
  server.begin();
  esp8266.tcpSetTimeout(SERVER_TIMEOUT);    // set x seconds timeout
  
  Serial.print(F("Server started! Go to "));
  Serial.println(esp8266.localIP());
  Serial.println(); 
}

//////////////////////////////////////////////////////////////////////////
///                    RFID SHIELD ROUTINES                             //
/////////////////////////////////////////////////////////////////////////

/* Gracefully handles a reader that is already configured and already reading continuously
 * Because Stream does not have a .begin() we have to do this outside the library
 * it could be handled, but wanted to keep SoftSerial for the moment */
void initialize_RFID()
{
  retry:
  
  Serial.println(F("Try to connect to RFID reader"));
  
  //Configure nano to run at certain speed
  int RunStatus = setupNano(RFID_SERIAL_SPEED);
  
  if (RunStatus == 0) 
  {
    serialTrigger(F("Module failed to respond. Please check wiring."));
    goto retry;
  }
  
  if (RunStatus == 1) // was not yet in continuous mode
  {
    nano.setRegion(RFID_Region); //Set to correct region

    // The M6E has this settings no matter what
    nano.setTagProtocol(); //Set protocol to GEN2

    // only one antenna connection available
    nano.setAntennaPort(); //Set TX/RX antenna ports to 1
    
    nano.setReadPower(RFID_Read_Power); //5.00 dBm. Higher values may caues USB port to brown out
    //Max Read TX Power is 27.00 dBm and may cause temperature-limit throttling as well as higher personal radiation 

    nano.startReading(); //Begin continuous scanning for tags
  }
  
  Serial.println("RFID shield initialized");
}

/*Gracefully handles a reader that is already configured and already reading continuously */
int setupNano(long baudRate)
{
  if (PRMDEBUG == 4) nano.enableDebugging(softSerial);
  
  nano.begin(softSerial); //Tell the library to communicate over software serial port
  
  //Test to see if we are already connected to a module
  //This would be the case if the Arduino has been reprogrammed and the module has stayed powered
  softSerial.begin(baudRate); //For this test, assume module is already at our desired baud rate

  while(!softSerial); //Wait for port to open

  //About 200ms from power on the module will send its firmware version at 115200. We need to ignore this.
  while(softSerial.available()) softSerial.read();

  val1 = 0;

  while(val1 < 2)
  {
    nano.getVersion();
    
    if (nano.msg[0] == ERROR_WRONG_OPCODE_RESPONSE )
    {
      if (PRMDEBUG > 1) Serial.println(F("Module continuously reading. Not Asking it to stop..."));
      return(2);
    }
  
    else if (nano.msg[0] != ALL_GOOD) 
    {
      if (PRMDEBUG > 1) Serial.println(F("Try reset RFID speed"));
    
      //The module did not respond so assume it's just been powered on and communicating at 115200bps
      softSerial.begin(115200); //Start software serial at 115200

      nano.setBaud(baudRate); //Tell the module to go to the chosen baud rate. Ignore the response msg

      softSerial.begin(baudRate); //Start the software serial port, this time at user's chosen baud rate
    }

    else
    {
      return(1);    // Repsonded, but have to be set in the right mode
    }
   
    val1++;
  }

  return(0);
}

/* try to read EPC from tag and add as unique to the array */
void Check_EPC()
{
  uint8_t myEPC[EPC_ENTRY];  // Most EPCs are 12 bytes
  
  if (nano.check() == true)  // Check to see if any new data has come in from module
  {
    byte responseType = nano.parseResponse(); //Break response into tag ID, RSSI, frequency, and timestamp

    if (responseType == RESPONSE_IS_KEEPALIVE)
    {
      if (PRMDEBUG > 1) Serial.println(F("Scanning"));
    }
    
    else if (responseType == RESPONSE_IS_TAGFOUND)
    {
      // extract EPC
      for (byte x = 0 ; x < EPC_ENTRY ; x++)
      {
        myEPC[x] = nano.msg[31 + x];
      }

      // add to array (if not already there)
      add_ECP_entry(myEPC, EPC_ENTRY);

      // display for debug information
      if (PRMDEBUG > 0)
      {
        //If we have a full record we can pull out the fun bits
        int rssi = nano.getTagRSSI(); //Get the RSSI for this tag read

        long freq = nano.getTagFreq(); //Get the frequency this tag was detected at

        long timeStamp = nano.getTagTimestamp(); //Get the time this was read, (ms) since last keep-alive message

        byte tagEPCBytes = nano.getTagEPCBytes(); //Get the number of bytes of EPC from response

        Serial.print(F(" rssi["));
        Serial.print(rssi);
        Serial.print(F("]"));

        Serial.print(F(" freq["));
        Serial.print(freq);
        Serial.print(F("]"));

        Serial.print(F(" time["));
        Serial.print(timeStamp);
        Serial.print(F("]"));

        //Print EPC bytes, 
        Serial.print(F(" epc["));
        for (byte x = 0 ; x < EPC_ENTRY ; x++)
        {
          if (myEPC[x] < 0x10) Serial.print(F("0")); //Pretty print adding zero
          Serial.print(myEPC[x], HEX);
          Serial.print(F(" "));
        }
        Serial.println("]");
      }
    }
    else
    {
      //Unknown response
      if (PRMDEBUG > 1) nano.printMessageArray(); //Print the response message. Look up errors in tmr__status_8h.html
    }
  }
}

/* initialize/empty the array to capture detected / unique EPC */
void init_array()
{
  for (val1 = 0 ; val1 < EPC_COUNT; val1++)  EPC_recv[val1][0] = 0;
}

/* add entry to array (if not there already) */
void add_ECP_entry(uint8_t *msg, byte mlength)
{
  byte j;
  val1=0;
  bool found;
  
  // as long as not end of list
  while(val1 < EPC_COUNT && EPC_recv[val1][0] != 0)
  {
    found = true;

    // check each entry for a match
    for (j = 0 ; j < mlength, j < EPC_ENTRY ; j++)
    {
      if (EPC_recv[val1][j] != msg[j]) 
      {
        found = false;  // indicate not fuond
        j = EPC_ENTRY;
      }
    }
   
    // if already there....
    if (found == true)
    {
      if (PRMDEBUG)
      {
        Serial.print(F("FOUND EPC in array entry "));
        Serial.println(val1);
      }
      return;
    }
    
    val1++;  // next entry
  }
  
  // check for available entries
  if (val1 == EPC_COUNT)
  {
    if (PRMDEBUG) Serial.println(F("Can not add more to array"));
    return;
  }
  
  // debug info
  if (PRMDEBUG)
  {
     Serial.print(F("New entry number is "));
     Serial.println(val1);
  }
  
  // add entry to the array
  for (j = 0 ; j < mlength, j < EPC_ENTRY ; j++)   EPC_recv[val1][j] = msg[j];
}

/* count number of entries with unique EPC received */
int count_entries()
{
  val1 = 0;
  
  while(val1 < EPC_COUNT)
  {
    if( EPC_recv[val1][0] == 0) break;
    val1++;
  }

  return(val1);
}

//////////////////////////////////////////////////////////////
////                   let's communicate                    //
//////////////////////////////////////////////////////////////

void Server_comms()
{
    int i = 0, j;

    int checkcnt = 0;             // only check connection every so often
    char input[40];               // place to hold command received
    int sock;                     // was there a new connection ???
 
  /* available() is an ESP8266Server function which will return an ESP8266Client object for printing and reading.
   * available() has one parameter -- a timeout value. This is the number of milliseconds the function waits,
   * checking for a connection. */
  ESP8266Client client = server.available(500, &sock);
  
  // if no real connection was found, check errors or return
  if (sock < 0 )
  {
    /* Did we loose the wifi connection / try to reconnect
     * this happens once in a while for unknown reasons. The response takes very long and then the ESP8266 shield
     * seems to have given up on his task.. */
    
    if (sock == -3) {
      
      for(i = 0 ; i < 2; i++)
      {
        Serial.println(F("lost connection : try to recover"));
        client = server.available(500, &sock);

        if (sock == -3)
        {
          initializeESP8266();
          connectESP8266();
          serverSetup();
        }
        else
          i = 2;
      }
    }

    return;
  }

  if (client) 
  {
    if (PRMDEBUG) Serial.println(F("Client Connected!"));

    // flush any pending input
    client.flush();
    
    // keep looping untill disconnect from client or time out
    while (1)
    {
      // try to read data from WIFI and skip header & retry 2 times
      i = readData(client, input, sizeof(input), 2);
      
      // if nothing received, check whether URL parameters have arrived
      // will only be provided one time.
      
      if (i == 0) j = client.get_url_parameter(input, sizeof(input));
      
      // if any data
      if (i > 0 || j > 0)
      {
  
        if (PRMDEBUG)
        {
          if ( i > 0 ) Serial.print(F("Read from remote: "));
          else Serial.print(F("Part of remote URL: "));
          Serial.println(input); 
        }

        handle_command(input, client);   // execute requested command
        
        if ( j > 0 ) break;              // got as part of URL break loop
        
        checkcnt = 1;                    // reset check counter

      }
      else
      {
        // to reduce overhead, only check if client is still connected every 5 loops
        // since no data was received
        if (checkcnt++ == 0) 
        {
            if (! client.connected()) break;
        }
        else if (checkcnt == 5 ) checkcnt = 0;   // trigger check for data
      }
     
      // try to read / add new tag (do not want to loose any)
      Check_EPC();
    }
   
    // close the connection:
    client.stop();
 
    // give the remote time to receive the data
    delay(1000);  
     
    if (PRMDEBUG)  Serial.println(F("Client disconnected"));
  }
}

/* send header-chunk and chunk data
 * out = data to send. make sure to reserver 3 places extra in the buffer for trailer
 * len = length of data in out
 * client = client instance
*/
void sent_chunk (char *out, int len, ESP8266Client client)
{
  char buf1[5];

  // add trailer to chunk data
  out[len] = '\r';
  out[len+1] = '\n';
  out[len+2] = 0x0;

  //send chunk-size header
  sprintf(buf1,"%x\r\n",len );
  client.print(buf1);
      
  //send chunk - data
  client.print(out);
}

/* break data to send into chunks
 * calculate chunk size, send chunk */
void do_chunk_output(char * mess, ESP8266Client client)
{
  int i=0, j=0;
  char output[30];      // need length + 3 (\r \n 0x0)
  int chunk_size;
  int len;
  
  len = strlen(mess);

  // calculate chunk size (a chunk with only \n or \r\n makes some browsers go bananas)
  if (len < 25) chunk_size = len;
  else if (len % 25 < 3) chunk_size = 25 - (len % 25);
  else chunk_size = 25;

  // as long as not all data send
  while(mess[i] != 0x0)
  {
      output[j++] = mess[i++];
      
      // if chunk size reached
      if (j == chunk_size)
      {
        sent_chunk(output, j, client);
        j = 0;
      }
  }

  if (j > 0) sent_chunk(output, j, client);
}

/* place to format and send the data to the client
 * mess = message to send
 * client = handle to client
 * http_cmd = true : format HTTP response / false : plain text
 */
void add_output(char * mess, ESP8266Client client, bool http_cmd)
{
  
  static bool header_sent = false;

  // I have put the definition here to enable F() (can only be used within a routine)
  const String htmlHeader1 = F("HTTP/1.1 200 OK\r\n"
                          "Content-Type: text/html\r\n"
                          "Connection: close\r\n"
                          "Transfer-Encoding: chunked\r\n");
  const String htmlHeader2 =F("<!DOCTYPE HTML>\r\n"
                          "<html>\r\n"
                          "<head>\r\n"
                          "<title>rfid reader</title>\r\n"
                          "<style>\r\n"
                          "body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\r\n"
                          "</style>\r\n"
                          "</head>\r\n"
                          "<body>\r\n" );
  const String trailer = F("</body>\r\n</html>\r\n");
  const String close_chunk = F("0\r\n\r\n");
  
  int r_length;
  static bool detect_closing_bracket = false;       // used during sending plain text.
  
  // add message to string
  reply += mess;
  
  // calculate length
  r_length = strlen(reply.c_str());
  
  // sent if a maximum storage space has been reached or empty message (= sent existing characters in buffer)
  // The maximum is set to be depending on local memory or remote buffer

  if (r_length > OUTMAX || strlen(mess) == 0)
  {
    if (http_cmd)
    {
      // if header has not been sent already
      if (header_sent == false) 
      {
        client.print(htmlHeader1);   // send header and indicate chunking
      
        do_chunk_output(htmlHeader2.c_str(), client);   // send chunked
    
        header_sent = true;
      }

      // any content pending to sent
      if (r_length > 0) do_chunk_output(reply.c_str(), client);
      
    }
    else    // plain text.. change HTML header/trailer to spaces
    {
      char *p = reply.c_str();
      char *q = p;
      
      while(*p != 0x0)
      { 
        // detect start of HTML 
        if ( *p == '<')  detect_closing_bracket = true;

       // end of HTML header ?
       if ( *p == '>' && detect_closing_bracket)
       {
            *p = 0x20;
            detect_closing_bracket = false;
       }
       
       // skip all within HTML header ( aso detect_closing_bracket might be left over from previous chunk)
       if (detect_closing_bracket) *p = 0x20;

       p++;
      }

      // send the updated buffer
      client.print(q);
    }
    
    // reset reply for next round
    reply = "";
  }

  // send trailer if empty message
  if (strlen(mess) == 0)
  {
    // only if header (and thus http content) was sent, send trailer
    if (header_sent)
    {
       do_chunk_output(trailer.c_str(), client);
       client.print(close_chunk.c_str());     // last chunk / trailer is send

       header_sent=false;
    }
    else
      // terminate plain text
      client.print("\r\n");
  }
}

/* handle the command received from remote 
 *  
 * get_cnt  : will return the number of EPC entries in the array
 * get_epc  : will return the EPCs discovered, comma seperated
 * clr_epc  : will clear the EPC table
 * 
 * the following command will be neglect it
 * CLOSED, favicon.ico
 * 
 * Multiple commands may be entered and will be handle in one returned message, separated by \n
 */
void handle_command(const char * command, ESP8266Client client)
{
  int i, j;
  char buf[40];                 // tmp for EPC translation
  reply = "";                   // reset output string
  bool valid = false;           // no valid command detected
  bool  http_cmd = false;       // false = return plain text
  
  // connection was close due to time out (neglect it)
  if (strstr(command,"CLOSED")) return;

  // connection was opened (neglect it, will be discovered in next loop)
  if (strstr(command,"CONNECT")) return;

  // request for a nice icon in front of the URL to be displayed (neglect it)
  if (strstr(command,"favicon")) return;

  // determine HTTP received command
  if (strstr(command,"HTTP")) http_cmd =true;
  
  // get EPC count
  if (strstr(command,"get_cnt"))
  {
    sprintf(buf,"<h1>get_cnt = %d</h1>",count_entries());
    add_output(buf, client, http_cmd);
    valid = true;                           // indicate a valid request was received
  }

  // get the detected EPC's
  if (strstr(command,"get_epc"))
  {
    add_output("<h1>get_epc = ", client, http_cmd);
    i = 0;
    
    // as long as there is a valid entry
    while ( EPC_recv[i][0] != 0)
    {
      // add comma in between EPC's
      if ( i > 0) add_output(",", client, http_cmd);

      // add epc
      for (j = 0 ; j < EPC_ENTRY; j++)
      {
        sprintf(buf, " %02x", EPC_recv[i][j]);
        add_output(buf, client, http_cmd);
      }

      // next EPC
      i++;
    }

    add_output("</h1>", client, http_cmd);
    valid = true;                       // indicate a valid request was received
  }

  // clear array command
  if (strstr(command,"clr_epc"))
  {
    init_array();
    add_output(" clr_epc = ok", client, http_cmd);
    valid = true;                      // indicate a valid request was received
  }

  // if not a valid requested
  if (valid == false)
  {
    add_output("Unknown request : ", client, http_cmd);
    add_output((char *) command, client, http_cmd);
    add_output("\n", client, http_cmd);
  }
  
  // sent trailer / closing message
  add_output("", client, http_cmd);
}

/* errorLoop prints an error code, then loops forever. */
void errorLoop(int error)
{
  Serial.print(F("Error: ")); Serial.println(error);
  esp8266.reset();
  
  Serial.println(F("Looping forever."));
  for (;;) if (esp8266.available())  esp8266.read();
}

/* serialTrigger prints a message, then waits for something
 * to come in from the serial port. */
void serialTrigger(String message)
{
  Serial.println();
  Serial.println(message);
  Serial.println();
  while (!Serial.available());
  while (Serial.available())Serial.read();
}

/* read data from remote/ WIFI and strip +IPD.... header
 *  
 * client = client socket
 * out = buffer to store output
 * len = length of buffer
 * retry = retry count for checking
 *
 * if len is zero, The read data after the header will be displayed
 *
 * return:
 * number of bytes received and stored in optional buffer
 */
int readData(ESP8266Client client, char *out, int len, int retry)
{
   int wait = 0;
   char c;
   int i = 0, j = 0, ch_count = 10;     // ch_count is starting at 10 to capture CLOSED (due to timeout)
   char ch_len[5];
   int retry_loop = retry;
  
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
        if (wait == 2) ch_len[j++] = c;  // after the second comma : store character count digit(s)

        if (c == ',') wait++;            // count comma's
        else if (c == ':')               // end of header
        {
          ch_len[j] = 0x0;               // terminate buffer
          ch_count= atoi(ch_len);        // calculate length of message
          wait = 3;                      // indicate header has been parsed
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
      else break;
    }
  }
  
  out[i] = 0x0;  // terminate
  
  // return count of characters in buffer
  return i;
}
