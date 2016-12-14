#ifndef __TCPIP__
#define __TCPIP__

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <arpa/inet.h>
#include <signal.h>

class Server{
private:
  struct sockaddr_in addr;
  struct sockaddr_in client;
  int sock;
  int sock0;

public:
  Server();
  ~Server();
  int Send(char*, unsigned int);
  int Recieve(char*, unsigned int);
};

class Client{
private:
  struct sockaddr_in server;
  int sock;

public:
  Client();
  ~Client();
  int Create_connection();
  int Send(char*, unsigned int);
  int Recieve(char*, unsigned int);
  int Close_connection();
};


#endif
