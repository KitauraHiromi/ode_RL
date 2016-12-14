#include "include/tcp.hpp"

Server::Server(){
  sock0 = socket(AF_INET, SOCK_STREAM, 0);
  if (sock0 < 0) {
    perror("socket");
  }

  addr.sin_family = AF_INET;
  addr.sin_port = htons(50007);
  addr.sin_addr.s_addr = INADDR_ANY;

  //サーバを切った後にTIME WAIT状態にしなくする
  int yes=1;
  setsockopt(sock0,SOL_SOCKET,SO_REUSEADDR,(const char *)&yes,sizeof(yes));

  if (bind(sock0, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
    perror("bind");
  }

  if (listen(sock0, 5) != 0) {
    perror("listen");
  }
}  

Server::~Server(){
  close(sock);
  close(sock0);
}

int Server::Send(char* str, unsigned int size){
  unsigned int n = write(sock, str, size);
  if (n < 1) {
    perror("write");
  }
  return n;
}

int Server::Recieve(char* str, unsigned int size){
  unsigned int len = sizeof(client);
  sock = accept(sock0, (struct sockaddr *)&client, (socklen_t *)&len);
  
  if (sock < 0) {
    perror("accept");
  }
  
  memset(str, ' ', size);
  unsigned int n = read(sock, str, size);
  return n;
}

Client::Client(){  
  /* host port setting */
  server.sin_family = AF_INET;
  server.sin_port = htons(50007);
  server.sin_addr.s_addr = inet_addr("127.0.0.1");
}

Client::~Client(){
  close(sock);
}

int Client::Create_connection(){
  /* creating socket */
  sock = socket(AF_INET, SOCK_STREAM, 0);
  
  /* connecting to server */
  connect(sock, (struct sockaddr *)&server, sizeof(server));
}

int Client::Send(char* str, unsigned int size){
  /* sending data to server */
  // ignore SIGPIPE
  signal( SIGPIPE , SIG_IGN );
  unsigned int n = write(sock, str, size);
  
  if (n < 1) {
    perror("write");
  }
  return n;
}

int Client::Recieve(char* str, unsigned int size){
  /* recieving data from server */
  memset(str, ' ', size);
  unsigned int n = read(sock, str, size);
  return n;
}

int Client::Close_connection(){
  close(sock);
}
