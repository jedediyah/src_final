#!/usr/bin/env python

### Field Computer Receiver
# This node simply listens for UDP packets and publishes the received data
# to whaler_UDP_received, which is subscribed to and parsed by the 
# field controller.  

import rospy
from std_msgs.msg import String
import socket 

import sys

def whaler_receive():

    # ROS  
    pub = rospy.Publisher('whaler_UDP_received',String, queue_size=50)
    thisnode = rospy.init_node('whaler_fc_receiver')

    # Networking
    sock = socket.socket(socket.AF_INET, # Internet
                         socket.SOCK_DGRAM) # UDP
    UDP_IP = "10.0.1.155"
    UDP_PORT = 5005
    print('Starting UDP server on port ' + str(UDP_PORT))
    sock.bind((UDP_IP, UDP_PORT))
            
    while not rospy.is_shutdown():
        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
        pub.publish(data)
        #print 'Received packet of size: ' + str(sys.getsizeof(data))
            

if __name__ == '__main__':
    try: 
        whaler_receive()
    except rospy.ROSInterruptException:
        rospy.loginfo('wfield_receiver ERROR')
