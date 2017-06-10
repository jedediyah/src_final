#!/usr/bin/env python

### Field Computer Control
# Controls Val's autonomy and listens for user commands from OCU

import rospy
from std_msgs.msg import String
from srcsim.msg import Satellite
from srcsim.srv import StartTask

import cv2
import numpy as np
#import matplotlib.pyplot as plt
from math import sqrt, isnan, sin, cos, atan, atan2, floor, pi
from listalgebra import *

import cPickle
import zlib
import socket 
from threading import Lock

from perception import PerceptionController
from valcontrol import KeyboardTeleop 

from communications import Transceiver
from communications import MessageMaker

import sys

PI = pi #3.14159265358979

class FieldController:
    def __init__(self):
        self.thisnode = rospy.init_node('whaler_fc_control')
        self.perception = PerceptionController()
        self.val = KeyboardTeleop()
        #self.val.init() 
        
        self.perception_update_needed = True 
        
        # A 'world' frame is implicitly created by initializing val's position and orientation 
        # to zero at startup, and then updating positions with transforms when val moves.  
        self.position_val = [[0.0,0.0,0.0]]  
        self.direction_val = [0.0]   # Radians  
        self.position_task1_dish = [[0.0,0.0,0.0]] 
        self.position_task1_controls = [[0.0,0.0,0.0]] 
        
        # The list of positions is used for update all positions as Val moves 
        self.positions = [self.position_val, self.position_task1_dish, self.position_task1_controls]
        
        # Track motion 
        self.motions = []   # rotations := ['rotation',angle], translations := ['translate',[dx,dy,dz]] where x is the forward direction
        
       
        # Commands received from OCU
        rospy.Subscriber('whaler_UDP_received', String, self.command_received)
        self.commands_to_run = []
        
        # Networking
        self.UDP_IP = '' #'10.0.1.155'
        self.UDP_PORT = 4004
        self.sock = socket.socket(socket.AF_INET,
                                  socket.SOCK_DGRAM)
                                  
        # Thread lock (don't know if this is necessary, but it's safer
        self.command_list_lock = Lock()
        
    
    def log(self,message):
        rospy.loginfo(message)
        self.transmit(message)
        
        
    ############
    ### MOTION 
    def rotate(self, angle):
        print "[rotate] Rotating: ", angle 
        self.perception_update_needed = True
        self.motions.append(['rotate',angle])
        self.direction_val.append( (self.direction_val[-1] + angle) % (2*PI) )
        self.log('Rotating: ' + str(angle))
        self.val.rotate(angle)
        rospy.sleep(1)      # If another step message is sent too soon, the wrong foot will move
    def step(self, distance):
        self.perception_update_needed = True
        self.motions.append(['translate',[-distance*sin(self.direction_val[-1]),0.0,distance*cos(self.direction_val[-1])]])  
        self.position_val.append( [self.position_val[-1][0] - distance*sin(self.direction_val[-1]), \
                                   self.position_val[-1][1], \
                                   self.position_val[-1][2] + distance*cos(self.direction_val[-1]) ])
        self.log('Stepping: ' + str(distance))
        self.val.translate([distance,0.0,0.0]) # X is the forward direction in the controller frame
        rospy.sleep(2)      # If another step message is sent too soon, the wrong foot will move

    ###############
    ### PERCEPTION    
    def updatePerception(self):
        if self.perception_update_needed:
            self.log('Updating perception')
            # TODO: make sure Val is not moving by checking /ihmc_ros/valkyrie/output/robot_motion_status as STANDING
            rospy.sleep(2) 
            self.perception.update()
            self.perception_update_needed = False
    def printMap(self):
        # Prints a plot and info aboutcurent object locations.
        print ('The following motions have been executed: ')
        for motion in self.motions:
            print motion 
        print 'Dish: ', self.position_task1_dish
        print 'Controls: ', self.position_task1_controls
        # TODO: Print some sort of direction indicator for Val 
        # Val
        # X = []
        # Z = [] 
        # for position in self.position_val:
        #     X.append(position[0])
        #     Z.append(position[2]) 
        # plt.plot(X,Z,'bo')
        # # Dish 
        # X = []
        # Z = [] 
        # for position in self.position_task1_dish:
        #     X.append(position[0])
        #     Z.append(position[2]) 
        # plt.plot(X[1:],Z[1:],'ro')
        # # Dish controls
        # X = []
        # Z = [] 
        # for position in self.position_task1_controls:
        #     X.append(position[0])
        #     Z.append(position[2]) 
        # plt.plot(X[1:],Z[1:],'go')
        # plt.axis('equal')
        # plt.show()
        
    def updatePosition_task1_dish(self):
        # Looks for the task1 dish in the Left head camera
        self.updatePerception()
        dish = self.perception.findDish()
        if len(dish) < 1:
            self.log("Task_1 dish not in view.")
            return False 
        valframe_dish = self.perception.getBlobLocation(dish)
        if isnan(valframe_dish[0]):     # Check if Val can see a dish 
            self.log("Task_1 dish not in view.")
            return False 
        t = self.direction_val[-1]
        ct = cos(t)
        st = sin(t)
        x = valframe_dish[0]
        y = valframe_dish[1]
        z = valframe_dish[2]
        worldframe_dish = [self.position_val[-1][0] + x*ct - z*st, \
                           self.position_val[-1][1] + y, \
                           self.position_val[-1][2] + x*st + z*ct]
        self.log('Task 1 dish observed at: ' + str([round(worldframe_dish[0],2), \
                                                         round(worldframe_dish[1],2), \
                                                         round(worldframe_dish[2],2)]))
        self.position_task1_dish.append(worldframe_dish)
        
    def updatePosition_task1_controls(self):
        # Looks for the task1 controls in the Left head camera
        self.updatePerception()
        controls = self.perception.findDishControls()
        if len(controls) < 1:
            self.log("Task_1 controls not in view.")
            return False 
        valframe_controls = self.perception.getBlobLocation(controls)
        if isnan(valframe_controls[0]):     # Check if Val can see controls 
            self.log("Task_1 controls not in view.")
            return False 
        t = self.direction_val[-1]
        ct = cos(t)
        st = sin(t)
        x = valframe_controls[0]
        y = valframe_controls[1]
        z = valframe_controls[2]
        worldframe_controls = [self.position_val[-1][0] + x*ct - z*st, \
                               self.position_val[-1][1] + y, \
                               self.position_val[-1][2] + x*st + z*ct]
        if sqrt((self.position_task1_dish[-1][0]-worldframe_controls[0])**2 + \
                (self.position_task1_dish[-1][1]-worldframe_controls[1])**2 + \
                (self.position_task1_dish[-1][2]-worldframe_controls[2])**2) < 0.8:
            # Likely misidentified the legs of the dish as the controls
            self.log("Task_1 controls not in view.")
            return False
        self.log('Task 1 controls observed at: ' + str([round(worldframe_controls[0],2), \
                                                             round(worldframe_controls[1],2), \
                                                             round(worldframe_controls[2],2)]))
        self.position_task1_controls.append(worldframe_controls)
    
    
    ##################
    ### MOTION CONTROL

    ###
    def turnTo(self, target_direction): 
        target_direction -= PI/2.0   # This offset comes from too many frame conversions...

        # Given a target direction, rotates Val in place to face that direction.
        print "[turnTo] Rotating from ", self.direction_val[-1], " to ", target_direction
        while abs(target_direction-self.direction_val[-1]) > PI/4:   
            if abs(target_direction-self.direction_val[-1]) < 0.1:
                pass
            elif target_direction > self.direction_val[-1]: 
                if target_direction - self.direction_val[-1] <= PI:
                    turndir =  1.0  # Turn left
                else:
                    turndir = -1.0  # Turn right
                self.rotate(turndir*PI/4)
            else:
                if self.direction_val[-1] - target_direction <= PI:
                    turndir = -1.0  # Turn right
                else:
                    turndir = 1.0   # Turn left 
                self.rotate(turndir*PI/4)
        if abs(target_direction-self.direction_val[-1]) < 0.1:
            pass
        elif target_direction > self.direction_val[-1]: 
            if target_direction - self.direction_val[-1] <= PI:
                turndir =  1.0  # Turn left
            else:
                turndir = -1.0  # Turn right
            self.rotate(turndir*abs(target_direction-self.direction_val[-1])) 
        else:
            if self.direction_val[-1] - target_direction <= PI:
                turndir = -1.0  # Turn right
            else:
                turndir = 1.0   # Turn left 
            self.rotate(turndir*abs(target_direction-self.direction_val[-1])) 
    
    ###
    def moveTo(self, target_position):
        # TODO: Project target position onto masked point_cloud of the path and use it
        # to path plan a way there while staying close to the middle of the path.
        # This maybe is best accomplished by breaking it into sub (recursive) moveTos. 
        #
        # target_position : [X,Y,Z] where from the camera's perspecive, +X is LEFT, +Y is UP, and +Z is FORWARD
        # In other words, moveTo() and turnTo() operate in the XZ-plane.  
        print "[moveTo] Moving to " + str(target_position)

        direction_vector = [ target_position[0]-self.position_val[-1][0], \
                             target_position[1]-self.position_val[-1][1], \
                             target_position[2]-self.position_val[-1][2] ]
        distance = lnorm( [ target_position[0]-self.position_val[-1][0] , target_position[2]-self.position_val[-1][2]  ] )
        target_direction = atan2(direction_vector[2],direction_vector[0])
        self.turnTo(target_direction) 
        
        # Walk to target location
        # TODO: Put in a double step first, so that Val isn't so bloddy slow. 
        stepcount = floor( distance/0.2 )
        while stepcount > 0:
            self.step(0.2)
            stepcount -= 1
        self.step( distance - floor(distance/0.2)*0.2 )  
            
        # TODO: Move this to a separate function, as it's not general enough to be here 
        # Once at the target location, turn to face the controls
        control_dish_vector = [self.position_task1_dish[-1][0]-self.position_task1_controls[-1][0], \
                               self.position_task1_dish[-1][1]-self.position_task1_controls[-1][1], \
                               self.position_task1_dish[-1][2]-self.position_task1_controls[-1][2] ]   # Vector from controls to dish
        direction_controls = atan2(control_dish_vector[2],control_dish_vector[0])
        self.turnTo(direction_controls) 
                    
        
    ###
    def getPathPlanForward(self):
        # Looks for path edges and generates a set of via points along a path in 
        # a direction away from previously visited points 
        via_points = []
        count, path_edgs = self.perception.get_mask_path_edge()
        if count != 2:
            return via_points 
        # Start from the bottom of the image and work up, 20 pixels at a time
        # TODO 
        return via_points 

    ###
    def showControlsImage(self):
        self.perception.readHeadCameras()
        left = self.perception.left_camera.copy()
        left_red = self.perception.findRedWheelPixels()
        left_blue = self.perception.findBlueWheelPixels()
        left_controls = self.perception.findDishControlsPixels()
        cv2.imshow('left',left)
        cv2.imshow('red',left_red)
        cv2.imshow('blue',left_blue)
        cv2.imshow('controls',left_controls)
        cv2.waitKey(0) 

    ###
    def getDistanceToControls(self):
        # ASSumes Val is standing directly in front of the Dish Controls AND is looking down
        # Looks for red and blue wheel, masks this onto the depth image and 
        # returns the average distance forward (Z-direction) in meters 
        self.updatePerception()
        red = self.perception.findRedWheelPixels()
        red_mask = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)
        blue = self.perception.findBlueWheelPixels()
        blue_mask = cv2.cvtColor(blue, cv2.COLOR_BGR2GRAY)
        pcl = self.perception.point_cloud  

        distances = []
        for row in xrange(red.shape[0]):
            for col in xrange(red.shape[1]):
                if red_mask[row][col]>1 or blue_mask[row][col]>1:
                    if not isnan(pcl[col+row*red.shape[1]][2]):
                        distances.append(pcl[col+row*red.shape[1]][2])
        result = np.mean(distances) 
        print "[getDistanceToControls] Val's distance to controls: ", result
        return result 

    ###
    def lookDown(self):
        print "Looking down"
        self.perception_update_needed = True 
        self.val.sendNeckTo([0.7,0.0,0.0])
        rospy.sleep(3) 
    ###
    def lookForward(self):
        print "Looking straight ahead"
        self.perception_update_needed = True 
        self.val.sendNeckTo([0.0,0.0,0.0])
        rospy.sleep(3) 


    def task1_rotate_red_wheel_clockwise(self):
        # ASSumes Val is well positioned in front of the dish controls
        print "Rotating RED wheel clockwise"

        # Move thumb down
        for i in range(6):
            self.val.process_key('4')   # TODO 

        A1 = -0.8
        A2 = -0.0
        self.val.sendArmTo('left',[A1,-1.3,1.6,A2,0.0,0.0,0.0])
        rospy.sleep(1.0)

        # Find left hand rectangle 
        self.perception.updateLeftCamera()
        rect_x,rect_y,rect_w,rect_h = self.perception.getRectangle( self.perception.findLeftHand() )

        # Find red wheel circle 
        circ_center,circ_radius = self.perception.getCircle( self.perception.findRedWheelPixels() )

        # Move the hand down so the top of the rectangle is at the middle of he circle
        vertical_hand_distance = rect_y+rect_h - circ_center[1]
        print "Vertical hand distance: ", vertical_hand_distance
        #while vertical_hand_distance > 0:
        print circ_center

        # Calculate horizontal distance in pixels between the left hand and the red wheel 
        distance = (circ_center[0]-circ_radius) - (rect_x+rect_w)
        print "Hand wheel distance (pixels): ", distance 

        # Move hand to wheel
        print 'Moving hand to wheel...'
        move_count = 0
        while distance > 0 and move_count < 15: # and delta_distance > 5:
            A2 -= 0.05
            self.val.sendArmTo('left',[A1,-1.3,1.6,A2,0.0,0.0,0.0])
            self.perception.updateLeftCamera() 
            rect_x,rect_y,rect_w,rect_h = self.perception.getRectangle( self.perception.findLeftHand() )
            circ_center,circ_radius = self.perception.getCircle( self.perception.findRedWheelPixels() )
            distance = (circ_center[0]-circ_radius) - (rect_x+rect_w)
            print "Hand wheel distance (pixels): ", distance 
            move_count += 1

            # Debugging image feedback
            hand = self.perception.left_camera.copy()
            cv2.circle(hand,circ_center,circ_radius,[0,220,0],2)
            cv2.rectangle(hand,(rect_x,rect_y),(rect_x+rect_w,rect_y+rect_h),[0,0,250])
            cv2.imshow('wheels',hand)
            cv2.imshow('red',self.perception.findRedWheelPixels())
            cv2.waitKey(100)

        # Press on wheel
        print 'Press on wheel...'
        A2 -= 0.15
        self.val.sendArmTo('left',[A1,-1.3,1.6,A2,0.0,0.0,0.0])

        # Rotate using friction 
        print 'Rotate wheel with friction...'
        A1 = -1.2
        self.val.sendArmTo('left',[A1,-1.3,1.6,A2,0.0,0.0,0.0])
#        self.val.sendArmTo('left',[-0.5,-1.3,1.6,-0.4,0.0,0.0,0.0])
        rospy.sleep(1.5)

        # Move arm back to start position 
        print 'Move arm back to start position...'
        A2 = -0.0
        self.val.sendArmTo('left',[A1,-1.3,1.6,A2,0.0,0.0,0.0])
        rospy.sleep(1.0)
        A1 = -0.8
        self.val.sendArmTo('left',[A1,-1.3,1.6,A2,0.0,0.0,0.0])
        rospy.sleep(1.5)


    ###
    def task1_rotate_red_wheel_counterclockwise(self):   
        # ASSumes Val is well positioned in front of the dish controls
        print "Rotating RED wheel counterclockwise"

        # Move thumb down
        for i in range(6):
            self.val.process_key('4')   # TODO 

        A1 = -1.0
        A2 = -0.0
        self.val.sendArmTo('left',[A1,-1.3,1.6,A2,0.0,0.0,0.0])

        # Find left hand rectangle 
        self.perception.updateLeftCamera()
        rect_x,rect_y,rect_w,rect_h = self.perception.getRectangle( self.perception.findLeftHand() )

        # Find red wheel circle 
        circ_center,circ_radius = self.perception.getCircle( self.perception.findRedWheelPixels() )

        # Move the hand down so the top of the rectangle is at the middle of he circle
        vertical_hand_distance = rect_y+rect_h - circ_center[1]
        print "Vertical hand distance: ", vertical_hand_distance
        #while vertical_hand_distance > 0:
        print circ_center

        # Calculate horizontal distance in pixels between the left hand and the red wheel 
        distance = (circ_center[0]-circ_radius) - (rect_x+rect_w)
        print "Hand wheel distance (pixels): ", distance 

        # Move hand to wheel
        print 'Moving hand to wheel...'
        move_count = 0
        while distance > 0 and move_count < 15: # delta_distance > 5:
            A2 -= 0.05
            self.val.sendArmTo('left',[A1,-1.3,1.6,A2,0.0,0.0,0.0])
            self.perception.updateLeftCamera() 
            rect_x,rect_y,rect_w,rect_h = self.perception.getRectangle( self.perception.findLeftHand() )
            circ_center,circ_radius = self.perception.getCircle( self.perception.findRedWheelPixels() )
            distance = (circ_center[0]-circ_radius) - (rect_x+rect_w)
            print "Hand wheel distance (pixels): ", distance 
            move_count += 1

            # Debugging image feedback
            hand = self.perception.left_camera.copy()
            cv2.circle(hand,circ_center,circ_radius,[0,220,0],2)
            cv2.rectangle(hand,(rect_x,rect_y),(rect_x+rect_w,rect_y+rect_h),[0,0,250])
            cv2.imshow('wheels',hand)
            cv2.imshow('red',self.perception.findLeftHand())
            cv2.waitKey(100)

        # Press on wheel
        print 'Press on wheel...'
        A2 -= 0.1
        self.val.sendArmTo('left',[A1,-1.3,1.6,A2,0.0,0.0,0.0])

        # Rotate using friction 
        print 'Rotate wheel with friction...'
        A1 = -0.8
        self.val.sendArmTo('left',[A1,-1.3,1.6,A2,0.0,0.0,0.0])
        rospy.sleep(1.5)

        # Move arm back to start position 
        print 'Move arm back to start position...'
        A2 = -0.3
        self.val.sendArmTo('left',[A1,-1.3,1.6,A2,0.0,0.0,0.0])
        rospy.sleep(1.0)
        A1 = -0.95
        A2 = -0.0
        self.val.sendArmTo('left',[A1,-1.3,1.6,A2,0.0,0.0,0.0])
        rospy.sleep(1.5)
        
        
    ###
    def task1_rotate_blue_wheel_clockwise(self):
        # ASSumes Val is well positioned in front of the dish controls
        print "Rotating BLUE wheel clockwise"

        # Move thumb down
        for i in range(6):
            self.val.process_key('7')   # TODO 

        A1 = -0.8
        A2 = 0.0
        self.val.sendArmTo('right',[A1,1.3,1.6,A2,0.0,0.0,0.0])
        rospy.sleep(1.0)

        # Find right hand rectangle 
        self.perception.updateLeftCamera()
        rect_x,rect_y,rect_w,rect_h = self.perception.getRectangle( self.perception.findRightHand() )

        # Find red wheel circle 
        circ_center,circ_radius = self.perception.getCircle( self.perception.findBlueWheelPixels() )

        # Move the hand down so the top of the rectangle is at the middle of he circle
        vertical_hand_distance = rect_y+rect_h - circ_center[1]
        print "Vertical hand distance: ", vertical_hand_distance
        #while vertical_hand_distance > 0:
        print circ_center

        # Calculate horizontal distance in pixels between the right hand and the red wheel 
        distance = (rect_x) - (circ_center[0]+circ_radius) 
        print "Hand wheel distance (pixels): ", distance 

        # Debugging image feedback
        hand = self.perception.left_camera.copy()
        cv2.circle(hand,circ_center,circ_radius,[0,220,0],2)
        cv2.rectangle(hand,(rect_x,rect_y),(rect_x+rect_w,rect_y+rect_h),[0,0,250])
        cv2.imshow('wheels',hand)
        cv2.imshow('blue',self.perception.findBlueWheelPixels())
        cv2.waitKey(100)

        # Move hand to wheel
        print 'Moving hand to wheel...'
        move_count = 0
        while distance > 0 and move_count < 15: # and delta_distance > 5:
            A2 += 0.05
            self.val.sendArmTo('right',[A1,1.3,1.6,A2,0.0,0.0,0.0])
            self.perception.updateLeftCamera() 
            rect_x,rect_y,rect_w,rect_h = self.perception.getRectangle( self.perception.findRightHand() )
            circ_center,circ_radius = self.perception.getCircle( self.perception.findBlueWheelPixels() )
            distance = (rect_x) - (circ_center[0]+circ_radius) 
            print "Hand wheel distance (pixels): ", distance 
            move_count += 1

            # Debugging image feedback
            hand = self.perception.left_camera.copy()
            cv2.circle(hand,circ_center,circ_radius,[0,220,0],2)
            cv2.rectangle(hand,(rect_x,rect_y),(rect_x+rect_w,rect_y+rect_h),[0,0,250])
            cv2.imshow('wheels',hand)
            cv2.imshow('blue',self.perception.findBlueWheelPixels())
            cv2.waitKey(100)

        # Press on wheel
        print 'Press on wheel...'
        A2 += 0.15
        self.val.sendArmTo('right',[A1,1.3,1.6,A2,0.0,0.0,0.0])

        # Rotate using friction 
        print 'Rotate wheel with friction...'
        A1 = -1.2
        self.val.sendArmTo('right',[A1,1.3,1.6,A2,0.0,0.0,0.0])
        rospy.sleep(1.5)

        # Move arm back to start position 
        print 'Move arm back to start position...'
        A2 = 0.0
        self.val.sendArmTo('right',[A1,1.3,1.6,A2,0.0,0.0,0.0])
        rospy.sleep(1.0)
        A1 = -0.8
        self.val.sendArmTo('right',[A1,1.3,1.6,A2,0.0,0.0,0.0])
        rospy.sleep(1.5)
        
    ###
    def task1_rotate_blue_wheel_counterclockwise(self):
        # ASSumes Val is well positioned in front of the dish controls
        print "Rotating BLUE wheel clockwise"

        # Move thumb down
        for i in range(6):
            self.val.process_key('7')   # TODO 

        # Send arm to start position [-0.8,-1.2,1.6,-0.2,0.0,0.0,0.0]
        self.val.sendArmTo('right',[-0.9,-1.3,1.6,-0.2,0.0,0.0,0.0])

        # Find right hand rectangle 
        self.perception.updateRightCamera()
        rect_x,rect_y,rect_w,rect_h = self.perception.getRectangle( self.perception.findRightHand() )

        # Find blue wheel circle 
        circ_center,circ_radius = self.perception.getCircle( self.perception.findBlueWheelPixels() )

        # Move the hand down so the top of the rectangle is at the middle of he circle
        vertical_hand_distance = rect_y+rect_h - circ_center[1]
        print "Vertical hand distance: ", vertical_hand_distance
        #while vertical_hand_distance > 0:
        print circ_center


        # Calculate horizontal distance in pixels between the right hand and the blue wheel 
        distance = (circ_center[0]-circ_radius) - (rect_x+rect_w)
        print "Hand wheel distance (pixels): ", distance 


        # Move hand to wheel
        print 'Moving hand to wheel...'
        while distance > 0:
            self.perception.updateRightCamera() 

            self.val.process_key('r')  # TODO: DON'T use these keys; send actual joint targets 
            #self.updatePerception()
            rect_x,rect_y,rect_w,rect_h = self.perception.getRectangle( self.perception.findRightHand() )
            circ_center,circ_radius = self.perception.getCircle( self.perception.findBlueWheelPixels() )
            distance = (circ_center[0]-circ_radius) - (rect_x+rect_w)
            print "Hand wheel distance (pixels): ", distance 

            hand = self.perception.right_camera
            cv2.circle(hand,circ_center,circ_radius,[0,220,0],2)
            cv2.rectangle(hand,(rect_x,rect_y),(rect_x+rect_w,rect_y+rect_h),[0,0,250])
            cv2.imshow('wheels',hand)
            cv2.waitKey(100)

        # Press on wheel
        print 'Press on wheel...'
        self.val.process_key('r')
        self.val.process_key('r')

        # Rotate using friction 
        print 'Rotate wheel with friction...'
        self.val.process_key('Q')
        self.val.process_key('Q')
        self.val.process_key('Q')
        self.val.process_key('Q')
        rospy.sleep(1.5)

        # Move arm back to start position 
        print 'Move arm back to start position...'
        self.val.sendArmTo('right',[-0.9,-1.3,1.6,-0.2,0.0,0.0,0.0])
        rospy.sleep(1.5)
        
        
    ############
    ### TASKS 
    
    ### Task 1 - Satellite Dish
    def task1(self):
        # TODO:
        # 1. Initial look for dish and controls
        # 2. Locate path edges and walk to just before the exit
        # 3. Face outward from middle of exit
        # 4. Locate dish and controls
        # 5. Step to target location 
        # 6. Rotate toward controls 

        print 'Executing task 1'

        ##############################################
        ## Find and approach satellite controls
        ## Searches for Dish and controls based on PCL data and controls colored pixels 
        self.updatePerception() 
        self.updatePosition_task1_dish()
        self.updatePosition_task1_controls()
        dish_position = self.position_task1_dish[-1]            # Not safe to just use most recent
        controls_position = self.position_task1_controls[-1]
        target_position = [-0.6*dish_position[0]+1.6*controls_position[0], \
                            0.0, \
                           -0.6*dish_position[2]+1.6*controls_position[2]]      # Hard coded values
        print "Target position: ", target_position
        self.moveTo(target_position)

        #############################################
        # Refine Val's position in front of controls
        # Assumes Val is standing rougly in front of the controls and facing the dish

        # Put arms out to avoid hitting anything while stepping
        self.val.sendArmTo('left',[-0.9,-1.0,1.6,0.0,0.0,0.0,0.0])
        self.val.sendArmTo('right',[-0.9,1.0,1.6,0.0,0.0,0.0,0.0])

        # Look down at controls 
        self.lookDown() 

        # Correct Rotation
        for i in range(2):
            self.perception.updateLeftCamera()
            red_center,red_radius = self.perception.getCircle( self.perception.findRedWheelPixels() )
            blue_center,blue_radius = self.perception.getCircle( self.perception.findBlueWheelPixels() )
            print red_center, blue_center
            if abs(red_center[1]-blue_center[1]) > 5:
                height = float(abs(red_center[1]-blue_center[1]))
                width = float(abs(blue_center[0]-red_center[0]))
                if red_center[1] > blue_center[1]:      # Rotate left
                    angle = 1.1*atan(height/width)
                else:
                    angle = -1.1*atan(height/width)
                self.rotate(angle)

        # Correct the lateral distance to controls 
        moving_laterally = True
        step_counter = 0
        step_divisor = 1000.0
        while moving_laterally:
            step_counter += 1
            if step_counter > 4:
                step_divisor *= 1.7 
                step_counter = 0
            rospy.sleep(2)
            self.perception.updateLeftCamera() 
            red_center,red_radius = self.perception.getCircle( self.perception.findRedWheelPixels() )
            blue_center,blue_radius = self.perception.getCircle( self.perception.findBlueWheelPixels() )
            d1 = red_center[0]
            d2 = 1023-blue_center[0]
            print d1, d2
            lateral_offset = abs(d2-d1)
            step_distance = lateral_offset/step_divisor
            print "Lateral offset: ", abs(d2-d1)
            if abs(d2-d1) < 10:
                moving_laterally = False 
                break 
            elif d2<d1:  # Move right
                self.val.step_right( min(step_distance,0.1) )
            else:
                self.val.step_left( min(step_distance,0.1) ) 

        # Correct the vertical distance to controls 
        distance_to_controls = self.getDistanceToControls() - 0.30
        while distance_to_controls > 0.2:
            self.step(0.2)
            distance_to_controls -= 0.2 
        if distance_to_controls >= 0.02:    # If the remaining distance is more than 2cm, take the step 
            self.step(distance_to_controls)


        # Correct dish position 
        self.val.sendArmTo('left',[-0.85,-1.2,1.6,-0.0,0.0,0.0,0.0])
        self.val.sendArmTo('right',[-0.85,1.2,1.6,0.0,0.0,0.0,0.0])
        
        # Left wheel
        #self.task1_fix_left()

    ###
    def task1_fix_left(self):
        sat_data = rospy.wait_for_message("/task1/checkpoint2/satellite", Satellite)
        while not sat_data.yaw_correct_now:
            if sat_data.target_yaw < sat_data.current_yaw:
                print 'less than'
                self.task1_rotate_red_wheel_clockwise()
            else:
                print 'greater than'
                self.task1_rotate_red_wheel_counterclockwise()
            sat_data = rospy.wait_for_message("/task1/checkpoint2/satellite", Satellite)
            
    ###
    def task1_fix_right(self):
        sat_data = rospy.wait_for_message("/task1/checkpoint2/satellite", Satellite)
        while not sat_data.pitch_correct_now:
            if sat_data.target_pitch < sat_data.current_pitch:
                print 'less than'
                self.task1_rotate_blue_wheel_clockwise()
            else:
                print 'greater than'
                self.task1_rotate_blue_wheel_counterclockwise()
            sat_data = rospy.wait_for_message("/task1/checkpoint2/satellite", Satellite)
            
    
    ###
    def run(self):
        #self.task1()
        self.task1_rotate_red_wheel_clockwise() 
        #self.testcode()
        self.val.fini()  # Shutdown Val controller 
        return 
        
    #####################################################################
    ### Below are May / June updates     
        
    def transmit(self, message):
        # Send a message to the OCU
        #msg = cPickle.dumps( message )
        if self.UDP_IP != '':
            msg = zlib.compress(cPickle.dumps( message ))
            print 'Transmitting message of size: ' + str(sys.getsizeof(msg)) 
            self.sock.sendto(msg,(self.UDP_IP,self.UDP_PORT))
        
        
    def command_received(self,data):
        # Callback function for subscriber to whaler_UDP_received
        # Unpack network data
        command = cPickle.loads(data.data)  
        self.log( "Received OCU command: " + str(command) )
        self.command_list_lock.acquire()
        self.commands_to_run.append(command)   # Append is a thread safe operation, but idk this feels safer
        self.command_list_lock.release()
        # Acknowledge receive of OCU command 
        self.transmit('acknowledge message '+str(command[0]))
        
    def run_semi_autonomous(self):
        self.log('Whalers running in semi-autonomous mode...')
        while True:
            if len(self.commands_to_run) <= 0:
                rospy.sleep(0.1)
                continue
            self.command_list_lock.acquire()
            command = self.commands_to_run.pop(0)
            self.command_list_lock.release()
            
            ##############################################################
            # An OCU command arrives as string in the form:
            # [command_id , command_type , argument]   #TODO: hash to check UDP integrity 
            # 
            # Parse command received from OCU
            if command[1] == 'X':
                # Emergency stop
                self.commands_to_run = [] 
            elif command[1] == 'ip':
                self.log('Setting OCU IP to ' + command[2])
                self.UDP_IP = command[2]  # A way of remotely setting the OCU ip 
            elif command[1] == 'init':
                self.val.init() 
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
                img = self.perception.updateLeftCamera()
                res = cv2.resize(img,None,fx=command[2], fy=command[2], interpolation = cv2.INTER_CUBIC)
                bw = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)
                #cv2.imshow('test',bw)
                #cv2.waitKey(0)
                self.transmit(['img',bw])
            elif command[1] == 't':
                # Perform a subroutine of a task
                if command[2] == '1':
                    self.task1() 
                    
            # MOTIONS 
            elif command[1] == 'rot':
                # Rotate by command[2]
                self.rotate(command[2])
            elif command[1] == 'step':
                self.step(command[2])
            elif command[1] == 'left':
                self.val.step_left(command[2])
            elif command[1] == 'right':
                self.val.step_right(command[2])
            elif command[1] == 'ldown':
                self.lookDown()
            elif command[1] == 'lup':
                self.lookForward()
                
            # TASKS
            elif command[1] == 'task':
                if command[2] == 'fixleft':
                    print 'Fixing left wheel...'
                    self.task1_fix_left()
            elif command[1] == 'T1':
                if command[2] == 'Lclock':
                    self.task1_rotate_red_wheel_clockwise()
                elif command[2] == 'Lcount':
                    self.task1_rotate_red_wheel_counterclockwise()
                elif command[2] == 'Rclock':
                    self.task1_rotate_blue_wheel_clockwise()
                elif command[2] == 'Rcount':
                    self.task1_rotate_blue_wheel_counterclockwise()
                
                
            
            

if __name__== "__main__":
    fieldControl = FieldController()
    #fieldControl.run()
    fieldControl.run_semi_autonomous()
    
    
    
    
    
