#ifndef VPSSOCKET_H
#define VPSSOCKET_H

#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>

// http://www.binarytides.com/winsock-socket-programming-tutorial/
#ifdef __WIN32
#include<winsock2.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <poll.h>
#include <netinet/in.h>
#endif

namespace vps {

// Define structures
#ifdef __WIN32
static WSADATA _wsa;
static SOCKET  _sockfd, _insockfd;
#else
static int socket _sockfd, _insockfd;
static struct pollfd _ufds;
#endif

static struct sockaddr_in _serv_addr, _cli_addr;
static int _clilen;
static bool _isopen;

void openConnection(int portno) {
#ifdef __WIN32
    if (WSAStartup(MAKEWORD(2,2),&_wsa) != 0) {
        printf("WASStartup Failed. Error Code : %d\n",WSAGetLastError());
        exit(-1);
    }
    if((_sockfd = socket(AF_INET , SOCK_STREAM , 0 )) == INVALID_SOCKET) {
        printf("Could not create socket : %d\n" , WSAGetLastError());
        exit(-1);
    }
    _serv_addr.sin_family = AF_INET;
    _serv_addr.sin_addr.s_addr = INADDR_ANY;
    _serv_addr.sin_port = htons(portno);

    if( bind(_sockfd ,(struct sockaddr *)&_serv_addr , sizeof(_serv_addr)) == SOCKET_ERROR) {
        printf("Bind failed with error code : %d\n" , WSAGetLastError());
        exit(-1);
    }

    listen(_sockfd, 3);
    printf("Waiting connection on port %d...\n", portno);

    _clilen = sizeof(struct sockaddr_in);
    _insockfd = accept(_sockfd, (struct sockaddr *)&_cli_addr, &_clilen);
    if(_insockfd == INVALID_SOCKET) {
        printf("Accept failed with error code : %d\n" , WSAGetLastError());
        exit(-1);
    }
    printf("Connection accepted!\n");
#else
    _sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (_sockfd < 0) {
        perror("Could not create socket!"); exit(-1); }

    memset((char *)&_serv_addr, '\0', sizeof(_serv_addr));
    _serv_addr.sin_family = AF_INET;
    _serv_addr.sin_addr.s_addr = INADDR_ANY;
    _serv_addr.sin_port = htons(portno);

    if (bind(_sockfd, (struct sockaddr *) &_serv_addr, sizeof(_serv_addr)) < 0) {
        printf("Bind failed!\n"); exit(-1); }

    _ufds.fd = _sockfd;
    _ufds.events = POLLIN;
    listen(_sockfd, 3);
    printf("Waiting connection on port %d...\n", portno);

    _clilen = sizeof(_cli_addr);
    _insockfd = accept(_sockfd, (struct sockaddr *) &_cli_addr, _&clilen);
    printf("Connection accepted!\n");
#endif
    _isopen = true;
}

void writeToConnection(void * data, int len) {
    if(!_isopen) return;
#ifdef __WIN32
    send(_insockfd, (char *)data, len, 0);
#else
    write(_insockfd, data, len);
#endif
}

void closeConnection() {
    std::cout << "Closing connections...\n";
#ifdef __WIN32
    closesocket(_sockfd);
    closesocket(_insockfd);
    WSACleanup();
#else
    close(_insockfd);
    close(_sockfd);
#endif
    _isopen = false;
    std::cout << "TCP server exited nicely\n";
}

} // namespace vps;

#endif // VPSSOCKET_H
