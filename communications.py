#!/usr/bin/env python

# This is the FIELD COMPUTER side of the transceiver

import struct
import socket
import cPickle
import zlib

import rospy
import roslib.message
import std_msgs.msg

import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image

RVIZ_FRAME = 'pelvis' 

class MessageMaker:
    # Translates between TCP data ROS messages

    ###
    def __init__(self):
        #self.joint_state_publisher = rospy.Publisher('/hardware_joint_states',JointState,queue_size=10)
        self.points2_publisher = rospy.Publisher('/multisense/camera/points2',PointCloud2,queue_size=10)

    ###
    def makeROS(self, data):
        # Given a list of the form [{message type (str)}, {message data (dict)}] received via TCP,
        # parse and create a ROS message.  
        if data[0] == '/hardware_joint_states':
            pass
        elif data[0] == '/tf':
            pass
        elif data[0] == '/multisense/camera/points2':
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = RVIZ_FRAME
            self.points2_publisher.publish(pcl2.create_cloud_xyz32(header,data[1]))
        elif data[0] == '/multisense/camera/left/image':
            pass
        else: 
            print ("WARNING!  Unrecognized data packet received.  Ignoring data")


class Transceiver:
    # A TCP transmitter and receiver
    
    ###
    def __init__(self):
        self.client = True 
        
    ###
    def startClient(self, ip_address, port):
        print ('Initializing network client...')
        self.ip_address = ip_address
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.ip_address, self.port))
        print ('Connected to ' + str(self.ip_address))
   
    ###
    def startServer(self,port):
        print ('Starting network server...')
        self.port = port 
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.bind(('', self.port))
        sock.listen(1)
        self.sock, self.addr = sock.accept()    # Initialize connection 
        print ('Connected to ' + str(self.addr))
        
    ###
    def stop(self):
        self.sock.close() 
        
    ###
    def transmit(self, data):
        msg = zlib.compress(cPickle.dumps( data ))  # Serialize and compress
        # Prefix each message with a 4-byte length (network byte order)
        msg = struct.pack('>I', len(msg)) + msg
        self.sock.sendall(msg)

    ###
    def receive(self):
        # Read message length and unpack it into an integer
        raw_msglen = self.recvall(4)
        if not raw_msglen:
            return None
        msglen = struct.unpack('>I', raw_msglen)[0]
        # Read the message data
        return cPickle.loads(zlib.decompress( self.recvall(msglen) ))
    
    ###
    def recvall(self, n):
        # Helper function to recv n bytes or return None if EOF is hit
        data = ''
        while len(data) < n:
            packet = self.sock.recv(n - len(data))
            if not packet:
                return None
            data += packet
        return data
        
        
