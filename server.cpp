/* A simple server in the internet domain using TCP
   The port number is passed as an argument */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include "server.h"

int sockfd, newsockfd, portno;
char read_buffer[256];
char write_buffer[256];

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

int create_socket()
{
     socklen_t clilen;

     struct sockaddr_in serv_addr, cli_addr;
     int n;

     sockfd = socket(AF_INET, SOCK_STREAM, 0);
     if (sockfd < 0)
         return -1;
        //error("ERROR opening socket");

     bzero((char *) &serv_addr, sizeof(serv_addr));
     portno = 3939; //atoi(argv[1]);
     serv_addr.sin_family = AF_INET;
     serv_addr.sin_addr.s_addr = INADDR_ANY;
     serv_addr.sin_port = htons(portno);

     if (bind(sockfd, (struct sockaddr *) &serv_addr,
              sizeof(serv_addr)) < 0)
         return -1;
             // error("ERROR on binding");


     listen(sockfd,5);
     clilen = sizeof(cli_addr);
     newsockfd = accept(sockfd,
                 (struct sockaddr *) &cli_addr,
                 &clilen);
     if (newsockfd < 0)
         return -1;
         // error("ERROR on accept");


//     bzero(buffer,256);
//     n = read(newsockfd,buffer,255);
//     if (n < 0) error("ERROR reading from socket");
//     printf("Here is the message: %s\n",buffer);
//     n = write(newsockfd,"I got your message",18);
//     if (n < 0) error("ERROR writing to socket");

     return 0;
}

int read_socket(char* data)
{
    int n = -1;
    bzero(data,256);
    n = read(newsockfd,data,255);
    if (n < 0)
        return -1;
        //error("ERROR reading from socket");
       return n;
}

int write_socket(char* data, size_t size)
{
    int n = -1;
    n = write(newsockfd, data, size);
    if (n < 0)
        return -1;
    return n;
}

void close_socket()
{
    close (newsockfd);
    close (sockfd);
}
