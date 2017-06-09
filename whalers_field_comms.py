#!/usr/bin/env python

### Field Computer

from communications import Transceiver
from communications import MessageMaker
import rospy

from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import point_cloud2 as pc2
from tf2_msgs.msg import TFMessage

class WhalerFieldComms:
    def __init__(self):
        # Initialize 
        self.thisnode = rospy.init_node('whaler_field_comms')
        self.comms = Transceiver()
        self.receiver = Transceiver()
        self.msgs = MessageMaker()
        self.comms.startServer(5005) # Waits for connection 
        #self.receiver.startClient('10.201.140.8',4004)
        
    def run(self):
        while True:
            ## Main loop
                        
            # Finals task
            # /srcsim/finals/task 
            
            # Joint state
            joint_state_msg = rospy.wait_for_message("/ihmc_ros/valkyrie/output/joint_states", JointState)
            #joint_state_msg = rospy.wait_for_message("/ihmc_ros/valkyrie/output/joint_states", JointState)
            self.comms.transmit( ['/ihmc_ros/valkyrie/output/joint_states', {'name':joint_state_msg.name, 'position':joint_state_msg.position}] )
            
            # Transforms 
            #transforms_msg = rospy.wait_for_message("/tf",TFMessage)
            #comms.transmit( ['/tf',{'transforms':transforms_msg.transforms}] )
            
            # Left camera image  
            # TODO: use grey scale instead of color to save bits?
            # TODO: or send Canny edge detection (presumably binary) to save bits
            left_camera_msg = rospy.wait_for_message('/multisense/camera/left/image_rect_color', Image)
            self.comms.transmit( ['/multisense/camera/left/image_rect_color',{'height':left_camera_msg.height, 'width':left_camera_msg.width, 'encoding':left_camera_msg.encoding, 'is_bigendian':left_camera_msg.is_bigendian, 'step':left_camera_msg.step,'data':left_camera_msg.data}] )
            
            # Point Cloud 
            # We don't send full point cloud, but the reduced colorless 3d points 
            pcl_msg = rospy.wait_for_message('/multisense/camera/points2', PointCloud2)
            #comms.transmit(['/multisense/camera/points2',{'height':pcl_msg.height, 'width':pcl_msg.width, 'is_bigendian':pcl_msg.is_bigendian, 'point_step':pcl_msg.point_step, 'row_step':pcl_msg.row_step, 'data':pcl_msg.data, 'is_dense':pcl_msg.is_dense}])
            pcl = pc2.read_points(pcl_msg, field_names= ("x", "y", "z"), skip_nans=True)
            pcl_data = []
            for point in pcl:
                pcl_data.append([point[2],point[0],point[1]])
            self.comms.transmit( ['/multisense/camera/points2', pcl_data] )
            
            rospy.sleep(0.5) 
            
        self.comms.stop()    


if __name__== "__main__":
    field = WhalerFieldComms()
    field.run() 
    
    
