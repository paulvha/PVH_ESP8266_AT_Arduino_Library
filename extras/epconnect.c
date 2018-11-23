/* epconnect.c  test client
 *
 * To be used in combination with the ESP8266_rfid_reader.ino 
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
 * To compile : cc -o epconnect epconnect.c
 * 
 * usage ./epconnect server_ipaddress
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

/* port to use on remote server 
 * must match the port defined by WIFI_PORT in sketch*/
#define PORT_TALK 80

/* buffer used to read from server */
#define MAXLINE 100
char    buff[MAXLINE];

char version[]="1.0 / paulvha / 2018";
char welcome[]="to remote: ";

void talkback(int sockfd,struct pollfd *fd);

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
            printf("\nStopping epconnect.\n");
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
    printf("\nusage %s  IP-address\n Version %s\n" ,name, version);
    exit(0);
}

int main(int argc,char ** argv)
{
    struct  hostent *he;
    struct  sockaddr_in server;
    struct  pollfd fd;   
    char    ip[30];
    
    if (argc < 2)  usage(argv[0]);

    strncpy(ip,argv[1],sizeof(ip));
    
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
    server.sin_port = htons(PORT_TALK);
    server.sin_addr = *((struct in_addr*)he->h_addr);
   
   if(connect(sockfd,(struct sockaddr *)&server,sizeof(server))== -1){
        printf("connect error\n");
        exit(1);
    }

    // set polling (check for input from remote)
    fd.fd = sockfd;
    fd.events = POLLIN;
    
    // start communicating with server
    talkback(sockfd, &fd);
    
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
    if (retry == 0) retry = 1;
    
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

/* Communicate with test with the Arduino server-demo */
void talkback(int sockfd, struct pollfd *fd)
{
    int ret, j, i = 0, header = 0;
    char c, input[25];
    
     // set at non-blocking keybard
    init_noblocking();
    
    // terminate input 
    input[0] = 0x0;
    
    printf("RFID reader demo \n");
    
    while(1)
    {
        if (header == 0) 
        {
            printf("\n%s%s",welcome, input);
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
            
            // prevent buffer overrun
            // send what is in it
            if (i == 24)
            {
                 input[i++] = '\n';
                 printf("\n");
            }
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
