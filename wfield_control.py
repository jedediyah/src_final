#!/usr/bin/env python

### Field Computer Receiver
import rospy
from std_msgs.msg import String

from srcsim.srv import StartTask  

import cPickle
import socket 
from threading import Lock

class whaler_field_controller:

    def __init__(self):

        # ROS
        self.thisnode = rospy.init_node('whaler_field_controller')
        rospy.Subscriber('whaler_UDP_received', String, self.command_received)
        
        # Commands from OCU
        self.commands_to_run = []
        
        # Networking
        self.UDP_IP = '10.0.1.155'
        self.UDP_PORT = 4004
        self.sock = socket.socket(socket.AF_INET,
                                  socket.SOCK_DGRAM)
                                  
        # Thread lock (don't know if this is necessary, but it's safer
        self.command_list_lock = Lock()
        
    def transmit(self, message):
        # Send a message to the OCU
        msg = cPickle.dumps( message )
        self.sock.sendto(msg,(self.UDP_IP,self.UDP_PORT))

        
    def command_received(self,data):
        # Callback function for subscriber to whaler_UDP_received
        # Unpack network data
        command = cPickle.loads(data.data)  
        rospy.loginfo( "Received OCU command: " + str(command) )
        self.command_list_lock.acquire()
        self.commands_to_run.append(command)   # Append is a thread safe operation, but idk this feels safer
        self.command_list_lock.release()
        # Acknowledge receive of OCU command 
        self.transmit('received message '+str(command[0]))


    def run(self):
        while True:
            if len(self.commands_to_run) <= 0:
                rospy.sleep(0.1)
                continue
            self.command_list_lock.acquire()
            command = self.commands_to_run.pop(0)
            self.command_list_lock.release()
            
            #################################
            # An OCU command arrives as string in the form:
            # [command_id , command_type , argument]   #TODO: hash 
            # 
            # Parse command received from OCU
            if command[1] == 'X':
                # Emergency stop
                self.commands_to_run = [] 
            elif command[1] == 's':     
                # Skip to a checkpoint
                # E.g.: [1234, 's', [1,3]] skips to Task 1 Point 3
                print 'Skipping to ', command[2][0], command[2][1]
                rospy.wait_for_service('/srcsim/finals/start_task')
                try:
                    skip_to = rospy.ServiceProxy('/srcsim/finals/start_task',StartTask)
                    skip_result = skip_to(command[2][0],command[2][1])
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e    
            elif command[1] == 'i':   
                # Transmit updated state and perception information back to OCU
                
                pass
                            
                
            
if __name__ == '__main__':
    Controller = whaler_field_controller()
    Controller.run()
    
    
    
