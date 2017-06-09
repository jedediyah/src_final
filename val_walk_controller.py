#!/usr/bin/env python

import time
import rospy
import tf
import tf2_ros
import numpy

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

from ihmc_msgs.msg import FootstepStatusRosMessage
from ihmc_msgs.msg import FootstepDataListRosMessage
from ihmc_msgs.msg import FootstepDataRosMessage


LEFT = 0
RIGHT = 1

ROBOT_NAME = None

class WalkController:

    def __init__(self):
        
        self.walk_msg = None
        self.last_side = RIGHT
        self.walk_started = False
        
        try:
            if not rospy.has_param('/ihmc_ros/robot_name'):
                rospy.logerr("Cannot run walk_test.py, missing parameters!")
                rospy.logerr("Missing parameter '/ihmc_ros/robot_name'")

            else:
                ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')

                right_foot_frame_parameter_name = "/ihmc_ros/{0}/right_foot_frame_name".format(ROBOT_NAME)
                left_foot_frame_parameter_name = "/ihmc_ros/{0}/left_foot_frame_name".format(ROBOT_NAME)

                if rospy.has_param(right_foot_frame_parameter_name) and rospy.has_param(left_foot_frame_parameter_name):
                    self.RIGHT_FOOT_FRAME_NAME = rospy.get_param(right_foot_frame_parameter_name)
                    self.LEFT_FOOT_FRAME_NAME = rospy.get_param(left_foot_frame_parameter_name)

                    footStepStatusSubscriber = rospy.Subscriber("/ihmc_ros/{0}/output/footstep_status".format(ROBOT_NAME), FootstepStatusRosMessage, self.recievedFootStepStatus)
                    self.footStepListPublisher = rospy.Publisher("/ihmc_ros/{0}/control/footstep_list".format(ROBOT_NAME), FootstepDataListRosMessage, queue_size=1)

                    self.tfBuffer = tf2_ros.Buffer()
                    self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

                    self.rate = rospy.Rate(10) # 10hz
                    self.motionSubscriber = None 
                    time.sleep(1)  # Not sure this is necessary

                    # make sure the simulation is running otherwise wait
                    if self.footStepListPublisher.get_num_connections() == 0:
                        #rospy.loginfo('waiting for subsciber...')
                        while self.footStepListPublisher.get_num_connections() == 0:
                            self.rate.sleep()

                    if not rospy.is_shutdown():
                        pass
                        #rospy.loginfo('Val walk controller ready.')
                        #walkTest()
                else:
                    if not rospy.has_param(left_foot_frame_parameter_name):
                        rospy.logerr("Missing parameter {0}".format(left_foot_frame_parameter_name))
                    if not rospy.has_param(right_foot_frame_parameter_name):
                        rospy.logerr("Missing parameter {0}".format(right_foot_frame_parameter_name))

        except rospy.ROSInterruptException:
            pass
            
    def leftStep(self):
        msg = FootstepDataListRosMessage()
        msg.transfer_time = 1.3
        msg.swing_time = 1.3
        msg.execution_mode = 0   
        msg.unique_id = -1
        msg.footstep_data_list.append(self.createFootStepOffset(LEFT, [0.2, 0.0, 0.0]))
        self.footStepListPublisher.publish(msg)
        rospy.loginfo('stepping forward with left...')
        self.waitForFootsteps(len(msg.footstep_data_list))  
        
    def leftFullStep(self, forwardOffset, transfer_time, swing_time):
        msg = FootstepDataListRosMessage()
        msg.transfer_time = transfer_time #0.75
        msg.swing_time = swing_time #0.85
        msg.execution_mode = 0   
        msg.unique_id = -1
        msg.footstep_data_list.append(self.createFootStepOffset(LEFT, [forwardOffset, 0.0, 0.0]))
        self.footStepListPublisher.publish(msg)
        rospy.loginfo('stepping forward with left...')
        self.waitForFootsteps(len(msg.footstep_data_list)) 
    
    def rightStep(self):
        msg = FootstepDataListRosMessage()
        msg.transfer_time = 1.3
        msg.swing_time = 1.3
        msg.execution_mode = 0   
        msg.unique_id = -1
        msg.footstep_data_list.append(self.createFootStepOffset(RIGHT, [0.2, 0.0, 0.0]))
        self.footStepListPublisher.publish(msg)
        rospy.loginfo('stepping forward with right...')
        self.waitForFootsteps(len(msg.footstep_data_list))  
        
    def rightFullStep(self, forwardOffset, transfer_time, swing_time):
        msg = FootstepDataListRosMessage()
        msg.transfer_time = transfer_time #0.75
        msg.swing_time = swing_time #0.85
        msg.execution_mode = 0   
        msg.unique_id = -1
        msg.footstep_data_list.append(self.createFootStepOffset(RIGHT, [forwardOffset, 0.0, 0.0]))
        self.footStepListPublisher.publish(msg)
        rospy.loginfo('stepping forward with right...')
        self.waitForFootsteps(len(msg.footstep_data_list))  
    
    ####################
    def initWalk(self,transfer_time, swing_time):
        self.walk_msg = FootstepDataListRosMessage()
        self.walk_msg.transfer_time = transfer_time
        self.walk_msg.swing_time = swing_time
        self.walk_msg.execution_mode = 0   # This is override mode, as opposed to 1 which is "queue mode"
        self.walk_msg.unique_id = -1
        
    def addStepToWalkPlan(self,delta, transfer_time, swing_time):
        if not self.walk_started:
            self.walk_started = True
            self.initWalk(transfer_time, swing_time)
        if self.last_side == LEFT:
            side = RIGHT
            self.last_side = RIGHT
        else:
            side = LEFT
            self.last_side = LEFT
        self.walk_msg.footstep_data_list.append(self.createFootStepOffset(side, [delta, 0.0, 0.0]))
        
    def go(self):
        self.walk_started = False
        self.footStepListPublisher.publish(self.walk_msg)
        rospy.loginfo('walking...')
        self.waitForFootsteps(len(self.walk_msg.footstep_data_list))   
            
    #####################
    
    def shuffleForward(self):
        self.leftStep()
        self.rightStep()
    
    #####################
            
    def walkToDoor(self):
        msg = FootstepDataListRosMessage()
        msg.transfer_time = 0.6
        msg.swing_time = 0.6
        msg.execution_mode = 0   # This is override mode, as opposed to 1 which is "queue mode"
        msg.unique_id = -1

        # walk forward starting LEFT
        msg.footstep_data_list.append(self.createFootStepOffset(LEFT, [0.3, 0.0, 0.0]))
        msg.footstep_data_list.append(self.createFootStepOffset(RIGHT, [0.6, 0.0, 0.0]))
        msg.footstep_data_list.append(self.createFootStepOffset(LEFT, [0.9, 0.0, 0.0]))
        msg.footstep_data_list.append(self.createFootStepOffset(RIGHT, [1.2, 0.0, 0.0]))
        msg.footstep_data_list.append(self.createFootStepOffset(LEFT, [1.5, 0.0, 0.0]))
        msg.footstep_data_list.append(self.createFootStepOffset(RIGHT, [1.8, 0.0, 0.0]))
        msg.footstep_data_list.append(self.createFootStepOffset(LEFT, [2.1, 0.0, 0.0]))
        msg.footstep_data_list.append(self.createFootStepOffset(RIGHT, [2.4, 0.0, 0.0]))
        msg.footstep_data_list.append(self.createFootStepOffset(LEFT, [2.7, 0.0, 0.0]))
        msg.footstep_data_list.append(self.createFootStepOffset(RIGHT, [3.0, 0.0, 0.0]))
        #msg.footstep_data_list.append(self.createFootStepOffset(LEFT, [3.3, 0.0, 0.0]))
        #msg.footstep_data_list.append(self.createFootStepOffset(RIGHT, [3.6, 0.0, 0.0]))
        #msg.footstep_data_list.append(self.createFootStepOffset(LEFT, [2.6, 0.0, 0.0]))
        #msg.footstep_data_list.append(self.createFootStepOffset(RIGHT, [2.8, 0.0, 0.0]))
        #msg.footstep_data_list.append(self.createFootStepOffset(LEFT, [3.0, 0.0, 0.0]))

        self.footStepListPublisher.publish(msg)
        rospy.loginfo('walking to door...')
        self.waitForFootsteps(len(msg.footstep_data_list))   
        
    def walkThroughDoor(self):
        msg = FootstepDataListRosMessage()
        msg.transfer_time = 1.3
        msg.swing_time = 1.3
        msg.execution_mode = 0   # This is override mode, as opposed to 1 which is "queue mode"
        msg.unique_id = -1

        # walk forward starting LEFT
        msg.footstep_data_list.append(self.createFootStepOffset(LEFT, [0.2, 0.0, 0.0]))
        msg.footstep_data_list.append(self.createFootStepOffset(RIGHT, [0.4, 0.0, 0.0]))
        msg.footstep_data_list.append(self.createFootStepOffset(LEFT, [0.6, 0.0, 0.0]))
        msg.footstep_data_list.append(self.createFootStepOffset(RIGHT, [0.8, 0.0, 0.0]))
        msg.footstep_data_list.append(self.createFootStepOffset(LEFT, [1.0, 0.0, 0.0]))
        msg.footstep_data_list.append(self.createFootStepOffset(RIGHT, [1.2, 0.0, 0.0]))
        msg.footstep_data_list.append(self.createFootStepOffset(LEFT, [1.4, 0.0, 0.0]))
        msg.footstep_data_list.append(self.createFootStepOffset(RIGHT, [1.6, 0.0, 0.0]))
        msg.footstep_data_list.append(self.createFootStepOffset(LEFT, [1.8, 0.0, 0.0]))
        msg.footstep_data_list.append(self.createFootStepOffset(RIGHT, [2.0, 0.0, 0.0]))
        
        #msg.footstep_data_list.append(self.createFootStepOffset(LEFT, [2.2, 0.0, 0.0]))
        #msg.footstep_data_list.append(self.createFootStepOffset(RIGHT, [2.4, 0.0, 0.0]))
        #msg.footstep_data_list.append(self.createFootStepOffset(LEFT, [2.6, 0.0, 0.0]))

        self.footStepListPublisher.publish(msg)
        rospy.loginfo('walking through door...')
        self.waitForFootsteps(len(msg.footstep_data_list))   

    # Creates footstep with the current position and orientation of the foot.
    def createFootStepInPlace(self, stepSide):
        footstep = FootstepDataRosMessage()
        footstep.robot_side = stepSide

        if stepSide == LEFT:
            foot_frame = self.LEFT_FOOT_FRAME_NAME
        else:
            foot_frame = self.RIGHT_FOOT_FRAME_NAME

        footWorld = self.tfBuffer.lookup_transform('world', foot_frame, rospy.Time())
        footstep.orientation = footWorld.transform.rotation
        footstep.location = footWorld.transform.translation

        return footstep

    # Creates footstep offset from the current foot position. The offset is in foot frame.
    def createFootStepOffset(self, stepSide, offset):
        footstep = self.createFootStepInPlace(stepSide)

        # transform the offset to world frame
        quat = footstep.orientation
        rot = tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
        #rot = tf.transformations.quaternion_matrix([1.0,0.0,0.0,0.0])    #    Don't use foot frame, walk in a STRAIGHT line...
        transformedOffset = numpy.dot(rot[0:3, 0:3], offset)
        
        #rospy.info(transformedOffset)

        footstep.location.x += transformedOffset[0]
        footstep.location.y += transformedOffset[1]
        footstep.location.z += transformedOffset[2]

        return footstep

    def waitForFootsteps(self, numberOfSteps):
        global stepCounter
        stepCounter = 0
        while stepCounter < numberOfSteps:
            self.rate.sleep()
        rospy.loginfo('finished set of steps')

    def recievedFootStepStatus(self, msg):
        global stepCounter
        if msg.status == 1:
            stepCounter += 1








