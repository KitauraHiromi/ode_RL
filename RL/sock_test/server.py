# -*- coding: utf-8 -*-

import socket
import threading
import time
import signal

LOCALHOST = "127.0.0.1"
PORT = 50007

class Server(threading.Thread):
    def __init__(self, host, port):
        super(Server, self).__init__()
        signal.signal(signal.SIGINT, self.handler)
        self.socket = socket.socket()
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((host, port))
        
    def __del__(self):
        try:
            self.conn.close()
        except AttributeError:
            pass
        self.socket.close()

    def handler(self, num, frame):
        self.__del__()
        
    def run(self):
        while True:
            self.socket.listen(1)
            self.conn, self.addr = self.socket.accept()  # クライアントの接続を待つ
            data = self.conn.recv(1024)  # クライアントからのデータを受信する
            print "server: receive '%s'" % data
            self.conn.send(data)  # クライアントにデータを送信する
        

class Client:
    def __init__(self, host, port):
        self.socket = socket.socket()
        self.socket.connect((host, port))
    
    def __del__(self):
        self.socket.close()
        
    def send(self):        
        self.socket.send("hello world")  # サーバにデータを送信する
        data = self.socket.recv(1024)  # サーバからのデータを受信する
        print "client: receive '%s'" % data

if __name__ == '__main__':
    s = Server(LOCALHOST, PORT)
    s.start()
    #c = Client(LOCALHOST, PORT)
    #c.send()
    #del s
    #del c
