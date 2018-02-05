/*
 * Copyright (c) 2018 Paul van Haastrecht.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * 
 * paulvha : January 2018 : version 1.0
 * 
 * This program sets as a server, will allow a client to connect over TCP and send
 * back the received information. It has been developed to work in conjunction with
 * pvh_ESP8266_Shield_client.ino part of the PVH_ESP8266_AT_Arduino_Library. 
 * Received commands CLOSE and STOP have special meaning
 *
 * Parts in the TCP connection code is taken from https://github.com/a1368017681/TCP
 * 
 * to compile : cc -o loop loop.c
 * to run     : ./loop
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <signal.h>
#include <getopt.h>
#include <ifaddrs.h>


#define VERSION "version 1.0 / January 2018 /paulvha "
#define PORT 333        // TCP port use
#define BACK_LOG 10     // maximum length to which the queue of pending connections
#define MAXLEN 512      // buffer size to use

// handles
int listenfd = 0 ,connectfd = 0;

/*********************************************************************
** close out program correctly 
**********************************************************************/
void closeout(int ret)
{
    if(listenfd > 0)    close(listenfd);
    if (connectfd > 0)  close(connectfd);
    exit(ret);
}

/*********************************************************************
** catch signals to close out correctly 
**********************************************************************/
void signal_handler(int sig_num)
{
    switch(sig_num)
    {
        case SIGKILL:
        case SIGABRT:
        case SIGINT:
        case SIGTERM:
            printf("\nStopping Loopback server\n");
            closeout(0);
            break;
        default:
            printf("\nneglecting signal %d.\n",sig_num);
    }
}

/*********************************************************************
** setup signals 
**********************************************************************/
void set_signals()
{
    struct sigaction act;
    
    memset(&act, 0x0,sizeof(act));
    act.sa_handler = &signal_handler;
    sigemptyset(&act.sa_mask);
    
    sigaction(SIGTERM,&act, NULL);
    sigaction(SIGINT,&act, NULL);
    sigaction(SIGABRT,&act, NULL);
    sigaction(SIGKILL,&act, NULL);
}

/****************************************************************** 
 * Display bytes in verbose
 * buff = input message to display
 * n    = input length of message
 * dir  = either receiving or sending
 * 
 ******************************************************************/
void disp_verbose(char *buff, int n, char * dir)
{
    int i;
    
    printf("%s ", dir);
    for (i = 0 ; i < n; i++) printf("%02x", (buff[i] & 0xff)); 
    
    printf("\n");
}

/****************************************************************** 
 * write bytes to the TCP remote client
 * buff = input message to write
 * n    = input length of message
 * 
 * Return
 * 0 = error
 * 1 = OK
 ******************************************************************/
int tcp_write (char *buff, int n)
{
    int ret, length = n;
    
    do
    {
        ret = write(connectfd,buff,length);
    
        if (ret == -1)
        {
            printf("Error during writing to TCP socket\n");
            return(0);
        }
    
        length -= ret;
        buff += ret;
    
    } while (length > 0);
    
    return(1);
}


/************************************************************** 
 *  read bytes from the TCP connection
 * *buff = ouptut message to received
 * *len  = output length of message received
 * 
 * Return
 *      0 = error
 *      1 = OK
 ***************************************************************/
int tcp_read(char *buff, int *len)
{
    int ret;

    ret = read(connectfd, buff, MAXLEN);
    
    if (ret == -1)
    {
        printf("Error during reading from TCP\n");
        return(0);
    }
    
    // terminate buffer
    buff[ret]=0x0;
    
    *len = ret;

    return(1);
}

/****************************************
 *  display usage and exit with failure 
 ****************************************/
 
void usage()
{
    printf("Usage: loop  [-v] \n"
           "       -v verbose: shows bytes exchanged\n\n"
           "%s\n\n",VERSION);
    
    exit(EXIT_FAILURE);
}

/**************************************************************
 * Main communication loop with client
 * 
 * connectfd = socket connection to remote client
 * verbose   = if set will display the bytes exchanged
 * ************************************************************/
void client_comm(int connectfd, int verbose)
{
    char buff[MAXLEN];
    int len = 0;
    int i,loop = 1;

    printf("ready to communicate\n");

    while (loop)
    {
        len = 0;
        
        // get command from TCP / client
        tcp_read(buff, &len);
        
        if (len == EOF || len == 0 )
        {
            loop  = 0;
            continue;
        }
        
        if (verbose) disp_verbose(buff, len, "from client");
        else printf("Received : ");
        
        for (i = 0; i < len ; i++)
        {
            if (buff[i] > 0x1f) printf("%c", buff[i]);
        }           
        printf("\n");
        
        if (strstr(buff, "CLOSE") != NULL)
        {
            printf("client decided to close connection for now\n");
        }
        
        // if STOP was sent, a message is displayed.
        // One could decide to close-down the parent of this process
        // and with that the server. It is not implemented however, who 
        // would let a client shutdown a server ????
        
        else if (strstr(buff, "STOP") != NULL)
        {
            printf("client stopped application\n");
        }
        else
        {
            if (verbose) disp_verbose(buff, len, "to client");
            else printf("now sending back: %s\n",buff);
        
            // write response to TCP / client
            if (loop && len > 0) loop = tcp_write(buff,len);
        }
    }
    
    printf("ending connection\n");
    
    closeout(EXIT_SUCCESS);
}

/* show connection details */
void show_connect()
{
    struct ifaddrs  *addrs, *tmp;
    char    ipa[20];
    
    getifaddrs(&addrs);
    tmp = addrs;

    while (tmp)
    {
        if (tmp->ifa_addr && tmp->ifa_addr->sa_family == AF_INET)
        {
            struct sockaddr_in *pAddr = (struct sockaddr_in *)tmp->ifa_addr;
            strcpy(ipa, inet_ntoa(pAddr->sin_addr));
            
            // skip 127.0.0.1 as that is a local no-no
            if (!(ipa[0] == '1' && ipa[1] == '2' && ipa[2] == '7') )
                printf("connect to %s:\t%s, port %d\n", tmp->ifa_name, ipa, PORT);
        }
    
    tmp = tmp->ifa_next;
    }

    freeifaddrs(addrs);
}

int main(int argc, char *argv[])
{
    struct sockaddr_in server;
    struct sockaddr_in client;
    pid_t childpid;
    socklen_t addrlen;
    int option, opt, verbose = 0 ;

    while ((opt = getopt(argc, argv, "v")) != -1)
    {
        switch (opt) 
        {
            case 'v':
              verbose = 1;
              break;
            default: /* '?' */
                usage();
        }
    }
 
    // catch signals
    set_signals();
    
    // open network connection
    listenfd = socket(AF_INET,SOCK_STREAM,0);
    
    if(listenfd == -1)
    {
        perror("TCP socket created failed");
        closeout(0);
    }

    option = SO_REUSEADDR;
    setsockopt(listenfd,SOL_SOCKET,option,&option,sizeof(option));
    
    bzero(&server,sizeof(server));
    server.sin_family = AF_INET;
    server.sin_port = htons(PORT);
    server.sin_addr.s_addr = htonl(INADDR_ANY);
    
    // bind to port
    if(bind(listenfd,(struct sockaddr *)&server,sizeof(server)) == -1)
    {
        perror("TCP Bind error!");
        closeout(EXIT_FAILURE);
    }
    
    // start listening
    if(listen(listenfd,BACK_LOG) == -1)
    {
        perror("TCP listening error");
        closeout(EXIT_FAILURE);
    }
    
    // display connect information
    show_connect();
    
    printf("\nwaiting for client's request.....\n");
    
    while(1)
    {
        addrlen = sizeof(client);
        connectfd = accept(listenfd,(struct sockaddr*)&client,&addrlen);
        
        if(connectfd == -1)
        {
            perror("TCP accept error");
            closeout(EXIT_FAILURE);
        }
        else
            printf("TCP client connected\n");

        // create child proces
        if((childpid = fork()) == 0)
        {   
            //child process to handle
            close(listenfd);
            listenfd = 0;
        
            printf("client from %s\n",inet_ntoa(client.sin_addr));
            
            // send as fast a possible
            option = 1;
            setsockopt(connectfd,IPPROTO_TCP,TCP_NODELAY,&option,sizeof(option));   

            client_comm(connectfd, verbose);
        }
        // parent process to handle
        else if(childpid < 0)
        {
            printf("fork error: %s\n",strerror(errno));
        
            // close connectfd in parent
            close(connectfd);
            connectfd = 0;
        }
    }
    
    // it should NEVER get here.... but does the software know that ??
    closeout(EXIT_SUCCESS);
}
