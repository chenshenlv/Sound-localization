# -*- coding: utf-8 -*-
"""
Created on Wed Jul 17 20:02:51 2019

@author: AskLan
"""
import socket
import json
import os

class client:
    '''demonstration class only
      - coded for clarity, not efficiency
    '''

    def __init__(self, sock=None):
        if sock is None:
            self.sock = socket.socket(
                socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.sock = sock

    def connect(self, host, port):
        self.sock.connect((host, port))

    def send(self, msg):
        self.sock.send(msg)
        
    def close(self):
        self.sock.close()
#        if sent == 0:
#            raise RuntimeError("socket connection broken")
#        totalsent = totalsent + sent

#    def myreceive(self):
#        chunks = []
#        bytes_recd = 0
#        while bytes_recd < MSGLEN:
#            chunk = self.sock.recv(min(MSGLEN - bytes_recd, 2048))
#            if chunk == '':
#                raise RuntimeError("socket connection broken")
#            chunks.append(chunk)
#            bytes_recd = bytes_recd + len(chunk)
#        return ''.join(chunks)

#local port: <1024 needs administrator
#ip_port = ('192.168.1.15', 8080)
#
##AF_INET: ipv4 SOCK_STREAM: TCP
#s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#
#s.connect(ip_port)
#
#print(s.recv(1024))
#
#while True:
#    msg = input('>>: ').strip()
#    
#    if len(msg) == 0:
#        continue
#    
#    if msg =='exit':
#        break
#    
#    s.send(msg.encode('utf-8'))
#
#    feedback = (s.recv(1024)).decode('utf-8')
#    print(feedback)
#    
#    feedback2 = (s.recv(1024)).decode('utf-8')
#    print(feedback2)
#    
#    file_name = feedback
#    file_size = feedback2
#    
#    msg2 = 'please transfer data'
#    s.send(msg2.encode('utf-8'))
#
#    new_file_name = "new " + file_name
#    f = open(new_file_name, "wb")
#
#    file_size = float(file_size)
#    num = file_size/1024.0
#    
#    if num != int(num):
#        num = int(num) +1
#    else:
#        num = int(num)
#    for i in range(num):
#        content = s.recv(1024)
#        print(content)
#        f.write(content)
#
#    f.close()
#
#
#s.close()
