#!/usr/bin/env python

# Test file for vpn communications

import socket

#REC_IP = "127.0.0.1"
REC_IP = '192.168.2.10'
REC_PORT = 5005

#SEND_IP = "127.0.0.1"
SEND_IP = '192.168.2.150'
SEND_PORT = 6006

sock_rec = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock_rec.bind((REC_IP, REC_PORT))

while True:
    data, addr = sock_rec.recvfrom(1024) # buffer size is 1024 bytes
    print "received message:", data
    sock_send = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
    sock_send.sendto(data, (SEND_IP, SEND_PORT))
