# -*- coding:utf-8 -*- 
import socket
import signal

class Server(object):
    def __init__(self, host, port):
        #super(Server, self).__init__()
        signal.signal(signal.SIGINT, self.handler)
        self.socket = socket.socket()
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((host, port))
        self.data = ''
        
    def __del__(self):
        try:
            self.conn.close()
        except AttributeError:
            pass
        self.socket.close()

    def handler(self, num, frame):
        self.__del__()

    def send(self, string):
        self.conn.send(string.upper())
        string = ''

    def recieve(self):
        self.socket.listen(1)
        self.conn, self.addr = self.socket.accept()  # waiting for client connection
        self.data = self.conn.recv(1024)   # receiving client data
        # print ("server: receive '%s'" % self.data)
        
class Client(object):
    def __init__(self, host, port):
        self.socket = socket.socket()
        self.socket.connect((host, port))
    
    def __del__(self):
        self.socket.close()
        
    def send(self):        
        self.socket.send("hello")  # サーバにデータを送信する
        data = self.socket.recv(1024)  # サーバからのデータを受信する
        # print "client: receive '%s'" % data
