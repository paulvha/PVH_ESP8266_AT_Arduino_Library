/* loopback, talkback and GPIO test client
 * 
 * This is part of the PVH-ESP8266_AT_Arduino library
 * 
 * To be used in combination with the pvh_ESP8266_Shield_loopback.ino or  
 * pvh_ESP8266_Shield_talkback.ino or pvh_ESP8266_Shield_SoftAP_gpio or
 * pvh_ESP8266_Shield_Server_gpio sketch (version 1.0)) on the Arduino
 * 
 * Copyright (c) 2018 Paul van Haastrecht <paulvha@hotmail.com>
 *
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the MIT License.
 *
 * This sofware is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * To compile : cc -o tclient tclient.c
 * 
 * usage ./tclient  [-t] [-l] ipaddress
 *
 *  -t     talkback test
 *  -l     loopback test
 *  -g     GPIO test
 *  
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <poll.h>
#include <termios.h>
#include <signal.h>
#include <getopt.h>

#define PORT_GPIO 330
#define PORT_LOOP 333
#define PORT_TALK 334

// match to sketch values for GPIO test Arduino
#define MAX_A_LED       '4'
#define MAX_ESP_LED     '2' 
#define MAX_A_BUTTON    '2'
#define MAX_ESP_BUTTON  '2'
#define MAX_ADC         '5'

#define MAXLINE 80
char    buff[MAXLINE];

char version[]="1.0 / paulvha / 2018";
char welcome[]="to remote: ";

void loopback(int sockfd,struct pollfd *fd);
void talkback(int sockfd,struct pollfd *fd);
void gpiotest(int sockfd, struct pollfd *fd);

struct termios new_settings;
struct termios old_settings;
int keyb_set = 0;
int sockfd;

/* closing out  */
void close_out()
{
    if (keyb_set == 1 ) tcsetattr(0,TCSANOW, &old_settings);
        
    close(sockfd);
    
    exit(0);
}

/* catch signals to close out correctly */
void signal_handler(int sig_num)
{
    switch(sig_num)
    {
        case SIGKILL:
        case SIGABRT:
        case SIGINT:
        case SIGTERM:
            printf("\nStopping tclient.\n");
            close_out();
            
            break;
        default:
            printf("\nNeglecting signal %d.\n",sig_num);
    }
}

/* setup signals */
void set_signals()
{
    struct sigaction act;
    
    memset(&act, 0x0,sizeof(act));
    act.sa_handler = &signal_handler;
    sigemptyset(&act.sa_mask);
    
    sigaction(SIGTERM,&act, NULL);
    sigaction(SIGINT,&act, NULL);
    sigaction(SIGABRT,&act, NULL);
    sigaction(SIGSEGV,&act, NULL);
    sigaction(SIGKILL,&act, NULL);
}

/* set for non-blocking  keyboard */
void init_noblocking()
{
    // save current setting
    tcgetattr(0,&old_settings);
    
    // set for noncononical
    new_settings=old_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    
    // no wait
    new_settings.c_cc[VMIN] =0;
    new_settings.c_cc[VTIME] =0;
    
    tcsetattr(0, TCSANOW, &new_settings);
    
    // indicate to restore on exit
    keyb_set = 1;
}

void usage(char *name)
{
    printf("\nusage %s  [-t] [-l] [-g] IP-address\n\n"
    "\t-t     talkback test\n"
    "\t-l     loopback test\n"
    "\t-g     GPIO test\n\n"
    "\tIP-address is of the server\n"
    "\nuse talkback in combination with pvh_ESP8266_Shield_talkback.ino\n"
    "use loopback in combination with pvh_ESP8266_Shield_loopback.ino\n"
    "use GPIO in combination with pvh_ESP8266_Shield_SoftAP_gpio or \n"
    "         pvh_ESP8266_Shield_Server_gpio\n"
    "\n\n%s\n" ,name, version);
    exit(0);
}

int main(int argc,char ** argv)
{
    struct  hostent *he;
    struct  sockaddr_in server;
    struct  pollfd fd;   
    int     c, loop=0, talk=0, gpio = 0;
    char    ip[30];

    while (1)
    {
        // parse the command line
        c = getopt(argc, argv,"-l:t:g:h");
            
        if (c == -1)    break;
            
        switch (c) {
            case 'l':   // loop back test
                if (talk == 1 || gpio == 1) usage(argv[0]);
                loop = 1;
                strncpy(ip,optarg,sizeof(ip));
                break;
                
            case 't':   // talk back test
                if (loop == 1 || gpio == 1) usage(argv[0]);
                talk = 1;
                strncpy(ip,optarg,sizeof(ip));
                break;
 
            case 'g':   // gpio test
                if (loop == 1 || talk == 1) usage(argv[0]);
                gpio = 1;
                strncpy(ip,optarg,sizeof(ip));
                break;                       
            
            default:
                usage(argv[0]);
                break;
        }
    }
    
    if (loop == 0 && talk == 0 && gpio == 0) usage(argv[0]);
    
    // catch cntrl-c to stop
    set_signals();

    he = gethostbyname(ip);
    
    if(he == NULL){
        printf("gethostbyname error\n");
        exit(1);
    }

    sockfd = socket(AF_INET,SOCK_STREAM,0);
    if(sockfd == -1){
        printf("socket error\n");
        exit(1);
    }

    bzero(&server,sizeof(server));
    server.sin_family = AF_INET;
    
    if (loop == 1)
        server.sin_port = htons(PORT_LOOP);
    else if (talk == 1)
        server.sin_port = htons(PORT_TALK);
    else if (gpio == 1)
        server.sin_port = htons(PORT_GPIO);   
    server.sin_addr = *((struct in_addr*)he->h_addr);
   
   if(connect(sockfd,(struct sockaddr *)&server,sizeof(server))== -1){
        printf("connect error\n");
        exit(1);
    }

    // set polling (check for input from remote)
    fd.fd = sockfd;
    fd.events = POLLIN;
    
    if (loop == 1) loopback(sockfd, &fd);
    else if (talk == 1) talkback(sockfd, &fd);
    else if (gpio == 1) gpiotest(sockfd, &fd);
    
    // stop -WALL complaining
    exit(0);
}   

/* read from remote connection
 * 
 * fd : poll fd
 * sockfd : socket
 * timeout : how long to wait
 * retry: how many times to retry
 * 
 * return
 * 
 * >0 : is number of bytes in buffer
 * 0  : is Nothing received
 * <0 : error
 */
int readRemote(struct pollfd *fd, int sockfd, int timeout, int retry)
{
    int ret;
    
    // try atleast 1 time
    if(retry == 0) retry = 1;
    
    // in case of failure, try x times to receive
    while(retry-- > 0)
    {
        ret = poll(fd,1,timeout);
        
        if (ret > 0)
        {   
            ret = read(sockfd, buff, MAXLINE);
            
            if (ret < 0)
            {
                printf("Error during reading from TCP\n");
                return -1;
            }
            
            else if (ret == 0)
            {
                printf("\nConnection disconnected\n");
                close_out();
            }
        
            // we are good
            else 
            {
                // terminate
                buff[ret] = 0x0;
                return ret;
            }
        }
        else if(ret < 0)
        {
            printf("Error during Polling\n");
            return -1;
        }   
    }
    
    return(0);
}

/* this a loopback test with the Arduino server-demo */
void loopback(int sockfd, struct pollfd *fd)
{
    int ret, retry;
    
    printf("Loop back demo \n");
    while(1)
    {
        printf("%s", welcome);
        fflush(stdout);
        
        // read message to sent
        fgets(buff,MAXLINE,stdin);
       
        retry = 0;
        
        // in case of failure, try 3 times to receive
        while(retry++ < 3)
        {
            printf("\nsending\n");
            
            if(write(sockfd,buff,strlen(buff)) < 0){
                printf("send msg error: %s \n",strerror(errno));
                break;
            }
    
            printf("waiting for echo\n\n");
            
            // now read feedback for setting
            // 2000 = 2 seconds
            // 3 = try 3 times
            ret = readRemote(fd, sockfd, 2000, 3);
        
            // we got some feedback
            if (ret > 0)  
            {
                printf("received : %s\n", buff); 
                break;
            }        
          
        }
    }
}

/* this a talkback test with the Arduino server-demo */
void talkback(int sockfd, struct pollfd *fd)
{
    int ret, j, i = 0, header = 0;
    char c, input[25];
    
     // set at non-blocking keybard
    init_noblocking();
    
    // terminate input 
    input[0] = 0x0;
    
    printf("Talking back demo \n");
    
    while(1)
    {
        if (header == 0) 
        {
            printf("%s%s",welcome, input);
            fflush(stdout);
            header = 1;
        }
        
        c = getchar();

        // control -c
        if (c == 0x3)   close_out();
        
        // backspace
        else if (c == 0x7f)
        {
            // if we have input
            if (i > 0)
            {
                // remove from screen
                printf("\b \b");
                fflush(stdout);
                
                // to previous position in input buffer
                i -= 1;
            }
        }
        
        // if a character received 
        else if (c != 0xff)
        {
            // to buffer
            input[i++] = c;
            
            //to screen
            printf("%c",c);
            fflush(stdout);
            
            // if enter or at max length of input
            if (c == '\n' || i == 25)
            {
                // terminate
                input[i] = 0x0;
       
                if(write(sockfd,input,strlen(input)) == -1)
                {
                    printf("send msg error: %s \n",strerror(errno));
                    break;
                }
                
                // display header 
                header = i = 0;
                
                // reset buffer display
                input[0] = 0x0;
            }
        }
        
        // check whether something was received from remote
        // 50 ms, try 1 time
        ret = readRemote(fd, sockfd, 50, 1);
        
        if (ret > 0)
        {
            // terminate received buffer
            buff[ret] = 0x0;

            // terminate input buffer
            input[i] = 0x0;
            
            // wipe out text on current line
            printf("\r");
            for(j=0; j<i+strlen(welcome); j++) printf(" ");
            
            // display received message
            printf("\rreceived : %s",buff);
            
            // set to display header
            header = 0;
        } // end received
    }// end while
}

/* This is a simple way to :
 *  1. set different LEDs (digital outputs) ON or OFF
 *  2. set the digital output to On/OFF at different blinking rate
 *  3. read the status of a button (digital Input) 
 *  4. read the level of an ADC (analog input)
 * 
 * action 1 - 3 can be performed on either the Arduino or ESP8266 AT Shield
 * 
 * action 4 can only be performed on the Arduino. Although the ESP8266 has ADC input
 * the loaded firmware does not support obtaining the value
 */ 
void gpiotest(int sockfd, struct pollfd *fd)
{
    int level, rate, act, brd, led, input, ret;
    char buf[30];
            
    // check we are connected
    // 500mS wait, 1 retry
    readRemote(fd, sockfd, 500, 1);
    
    while (1)
    {    
        printf("\nWhat action do you want to do ?\n"
        "1  switch a led on or off\n"
        "2  set a led for blinking\n"
        "3  read a button\n"
        "4  read an ADC value\n"
        "9  exit\n");
        
        act = 0;
        while(act == 0)
        {
            act = getchar();
            
            if (act == '9') close_out();
            
            if (act < '1' || act > '4') act = 0;
        }
        
        // from Ascii
        act = act - 0x30;
        
        // we can NOT use the ADC from the ESP8266
        if (act != 4)
        {
            printf("\nOn which board is it connected ?\n"
            "1  Arduino\n"
            "2  ESP8266\n"
            "9  exit\n");
            
            brd = 0;
            while(brd == 0)
            {
                brd = getchar();
                
                if (brd == '9') close_out();
                
                if (brd != '1' && brd != '2') brd = 0;
            }
            
            // from ascii
            brd = brd - 0x30;
            
            if ( act == 1 || act == 2 )
            {
                printf("\nWhich LED ?\n");
                
                led = 0;
                while(led == 0)
                {
                    led = getchar();
                    
                    // Arduino
                    if (brd == 1) 
                    {
                        if (led < '0' || led > MAX_A_LED) led = 0;
                    }
                    else // ESP board
                    {
                        if (led < '0' || led > MAX_ESP_LED ) led = 0;
                    }
                }
    
                if (act == 2)
                {
                    printf("\nHow many seconds do you want the led On/Off?\n (1 - 9)");
                    
                    rate =0;
                    while(rate == 0)
                    {
                        rate = getchar();
                        
                        if (rate < '1' || rate > '9') rate = 0 ;
                    }
                    
                    // calculate the blinking rate
                    rate = 60 / (rate-0x30);
                }
                else
                {
                    printf("\nWhat level to set LED%c ?\n"
                    "1  ON\n"
                    "2  OFF\n"
                    "9  exit\n", led);
                    
                    level = 0;
                    while(level == 0)
                    {
                        level = getchar();
                        
                        if (level == '9') close_out();
                        
                        if (level != '1' && level != '2') level =0 ;
                    }
                }
                
                // 0 = will set to ON, 1 = will set to OFF
                level = level - 0x30 -1;       
                            
            }
        }
        
        // read an Analog or digital Pin
        if (act == 3 || act == 4)
        {
            printf("\nWhich input pin?\n");
            
            input = 0;
            while(input == 0)
            {
                input = getchar();
                
                // get input from digital pin
                if (act == 3)
                {
                    // Arduino
                    if (brd == 1) 
                    {
                        if (input < '0' || input > MAX_A_BUTTON) input = 0;
                    }
                    else // ESP8266
                    {
                        if (input < '0' || input > MAX_ESP_BUTTON ) input = 0;
                    }
                }
                // get input from analog pin
                else
                {
                    if (input < '0' || input > MAX_ADC) input = 0;
                }
            }
        }
    
        // if switch or blink
        if (act == 1 || act ==2)
        {
            if (brd == 1) 
            {
                if (act == 1) sprintf(buf,"SWITCH(A_LED%c,%s)",led, level ? "OFF":"ON");
                else sprintf(buf,"BLINK(A_LED%c,%d)",led, rate);
            }
            else
            {
                 if (act == 1) sprintf(buf,"SWITCH(ESP_LED%c,%s)",led, level ? "OFF":"ON");
                else sprintf(buf,"BLINK(ESP_LED%c,%d)",led, rate);
            }
        }
        
        // if requesting status of digital input
        else if (act == 3)
        {
            // arduino
            if (brd == 1)  sprintf(buf,"STATUS(A_BUT%c)",input);
            else  sprintf(buf,"STATUS(ESP_BUT%c)",input);
        }
        
         // if requesting ADC input
        else if (act == 4)
        {
            sprintf(buf,"ADC(A_%c)",input);
        }
        
        printf("\nSending: %s\n\n", buf);
     
        if(write(sockfd,buf,strlen(buf)) < 0)
        {
            printf("send msg error: %s \n",strerror(errno));
        }
        
        // now read feedback for setting
        // 2000 = 2 seconds
        // 3 = try 3 times
        ret = readRemote(fd, sockfd, 2000, 3);
        
        if (ret == 0)
        {
            printf("Nothing received\n\n");
        }
        else
        {
            if (act == 1 || act == 2) 
            {
                printf("received feedback: %s\n\n",buff);
            }
            else if (act == 3)
            {
                printf("status for Digital pin %c: %s\n\n",input, buff);
            }
            else if (act == 4)
            {
                printf("Level for ADC pin %c: %s\n\n",input, buff);
            }
        }
    }
}
