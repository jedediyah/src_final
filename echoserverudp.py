#!/usr/bin/env python

# Test file for vpn communications

import socket
from std_msgs.msg import String 
import rospy
import time 

# WAIT for stuff to start up I guess???
print 'Waiting for roscore to start...'
time.sleep(10)
print 'Starting server'

#REC_IP = "127.0.0.1"
REC_IP = '192.168.2.10'
REC_PORT = 5005

#SEND_IP = "127.0.0.1"
SEND_IP = '192.168.2.150'
SEND_PORT = 6006

# ROS  
thisnode = rospy.init_node('whaler_fc_receiver')
pub = rospy.Publisher('whaler_UDP_received',String, queue_size=20)

# Sockets
sock_rec = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock_rec.bind((REC_IP, REC_PORT))

sock_send = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
while True:
    data, addr = sock_rec.recvfrom(1024) # buffer size is 1024 bytes
    print "received message:", data
    sock_send.sendto(data, (SEND_IP, SEND_PORT))
    pub.publish(data)
    
 
   
            
            
            
