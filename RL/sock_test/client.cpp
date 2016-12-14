#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <arpa/inet.h>

int main(){
  struct sockaddr_in server;
  int sock;
  char buf[32];
  int n;
  double count = 0;

  while(1){
    
    /* ソケットの作成 */
    sock = socket(AF_INET, SOCK_STREAM, 0);
    
    /* 接続先指定用構造体の準備 */
    server.sin_family = AF_INET;
    server.sin_port = htons(50007);
    server.sin_addr.s_addr = inet_addr("127.0.0.1");
    
    /* サーバに接続 */
    connect(sock, (struct sockaddr *)&server, sizeof(server));

    /* サーバにデータを送信 */
    char send_buf[256];
    memset(send_buf, ' ', sizeof(send_buf));
    sprintf(send_buf, "%lf %lf %lf %lf %lf", count, count, count, count, count);
    n = write(sock, send_buf, sizeof(send_buf));
    if (n < 1) {
      perror("write");
      break;
    }

    count += 1;
    
    /* サーバからデータを受信 */
    memset(buf, 0, sizeof(buf));
    n = read(sock, buf, sizeof(buf));
    printf("%d, %s\n", n, buf);

    /* socketの終了 */
    close(sock);
  }

  return 0;
}

