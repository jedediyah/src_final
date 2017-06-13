#!/usr/bin/env python

# Test file for vpn communications

import socket

#SEND_IP = '127.0.0.1'
SEND_IP = '192.168.2.10'
SEND_PORT = 5005

#REC_IP = '127.0.0.1'
REC_IP = '192.168.2.150'
REC_PORT = 6006

MESSAGE = "Testing this"

sock_send = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock_send.sendto(MESSAGE, (SEND_IP, SEND_PORT))


sock_rec = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock_rec.bind((REC_IP, REC_PORT))

data, addr = sock_rec.recvfrom(1024) # buffer size is 1024 bytes
print "received message:", data


