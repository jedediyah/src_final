#!/usr/bin/env python

from cv2_addons import drawMatches

import rospy
import tf
import tf.transformations
import cv2
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
import point_cloud2 as pc2
import roslib.message
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage

from math import sqrt, isnan

import time
import cPickle
import zlib

class PerceptionController:
    ### Important aspects of camera perception:
    # 1. Identify path 
    # 2. Identify target objects

    
    ###
    def __init__(self):
        print "Initializing perception..."
        # Head sensors
        self.left_camera = None
        self.right_camera = None
        self.point_cloud = None  # In pelvis frame
        
        # Processed images
        self.depth_image = None
        self.path_contour = None
        self.pcl_path = None            # Masked pcl identifying path for walking
        self.pcl_task1_dish = None      # Masked pcl identifing communications dish
        self.pcl_task1_controls = None  # Masked pcl identifing comms dish controls 

        self.left_filtered = None 
        
        # Visual histories of object identification
        self.path_ids = []
        self.task1_control_ids = []
        
        #self.update()
        print "Done."
        
    ###
    def saveImage(self,img,filename):
        # TODO: include timestamp
        compressed = zlib.compress(cPickle.dumps(img))
        with open(filename,'wb') as pfile:
            cPickle.dump(compressed, pfile,-1)
        return
    ###
    def loadImage(self,filename):
        with open(filename, 'rb') as input:
            img = cPickle.loads(zlib.decompress( cPickle.load(input) ))
        return img
        
    ###
    def update(self):
        # Reads head cameras and lidar and does image processing for other functions can use
        self.readHeadCameras() 
        self.readPointCloud() # TODO: do a running average for PCL data
        self.left_filtered = self.removeBackground(self.maskLeftCamera(self.left_camera.copy()))
        
        
    ###############
    ## Point Cloud
    
    ###
    def readPointCloud(self):
    
        # TODO: self.point_cloud is currently a 544*1024 single dimensional array of [x,y,z] data.  
        # We could more easily write some optimized functions if it were broken down into 
        # three (X,Y,Z) arrays, each containing the [row][col][distance] for that dimension.  
    
        pc_message = rospy.wait_for_message("/multisense/camera/points2", PointCloud2) 
        self.point_cloud = pc2.read_points(pc_message, field_names = ("x", "y", "z"), skip_nans=False)
        
        # Get transform from pelvis frame
        listener = tf.TransformListener()   
        trans = []
        quat = []
        rate = rospy.Rate(10.0)
        while not trans:
            try:
                (trans,quat) = listener.lookupTransform('/pelvis', '/head_hokuyo_frame', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()
        angles = tf.transformations.euler_from_quaternion(quat)
        quat = tf.transformations.quaternion_from_euler(angles[1],angles[0],angles[2])   # TODO: what is going on here?
        rt = tf.transformations.inverse_matrix( tf.transformations.quaternion_matrix( quat ) ) 
        
        # Transform pcl into pelvis frame, but not really for some reason
        pcl_data = 544*1024*[[np.nan,np.nan,np.nan]]
        index = 544*1024
        for p in self.point_cloud:
            index -= 1
            if not isnan(p[2]): 
                x = rt[0][0]*p[0] + rt[0][1]*p[1] + rt[0][2]*p[2] #-trans[0]   # TODO: these translations are wrong
                y = rt[1][0]*p[0] + rt[1][1]*p[1] + rt[1][2]*p[2] #-trans[1] 
                z = rt[2][0]*p[0] + rt[2][1]*p[1] + rt[2][2]*p[2] #-trans[2]  
                pcl_data[index]=[x,y,-z]    # TODO: Why oh why are transformations so difficult?  
        self.point_cloud = pcl_data
        return self.point_cloud
        
        
    ###
    def getDepthImage(self):
        depth_image = np.zeros((544,1024,3), np.uint8)
        row=col=0
        for point in self.point_cloud: 
            if not isnan(point[2]): 
                distance = sqrt(point[0]**2+point[1]**2+point[2]**2)
                depth_image[row,col][1] = 255-min(255,25*distance) 
                depth_image[row,col][2] = 255
            col += 1
            if col >= 1024:
                col = 0
                row += 1
        self.depth_image = depth_image
        return depth_image
        #return cv2.flip(cv2.flip( depth_image,0),1)
        
    ###
    def getMaskedDepthImage(self):
        # Get camera image
        left_filtered = self.left_filtered 
        
        # Get point cloud
        pcl_img = self.getDepthImage() 
        #cv2.imshow('unmasked',pcl_img)
        
        # Mask point cloud with camera image 
        lower_threshold = np.array([0,0,0])
        upper_threshold = np.array([200,200,100])
        mask = cv2.inRange(left_filtered, lower_threshold, upper_threshold)
            
        # Apply mask
        res = cv2.bitwise_and(pcl_img,pcl_img, mask=mask)
        #cv2.imshow('img',res)
        #cv2.waitKey(0)
        return res 
    
    ###
    def pcl_threshold_distance(self,lower,upper):
        lower_squared = lower**2
        upper_squared = upper**2
        depth_image = np.zeros((544,1024,3), np.float32)   # TODO: use uint8
        row=col=0
        for point in self.point_cloud:
            if not isnan(point[2]): 
                distance_squared = point[0]**2+point[1]**2+point[2]**2
                if distance_squared >= lower_squared and distance_squared <= upper_squared:
                    depth_image[row,col][2] = 1.0
                    depth_image[row,col][1] = 1-min(1.0,sqrt(distance_squared)/15.0) 
            col += 1
            if col >= 1024:
                col = 0
                row += 1
        return depth_image #cv2.flip(cv2.flip( depth_image,0),1)
        
    ###
    def pcl_threshold_height(self,lower,upper):
        lower_squared = lower**2
        upper_squared = upper**2
        depth_image = np.zeros((544,1024,3), np.uint8)
        row=col=0
        for point in self.point_cloud:
            if not isnan(point[2]): 
                distance_squared = point[0]**2+point[1]**2+point[2]**2
                if distance_squared < 81 and point[1] < upper and point[1] > lower :  # Assumption: only looking within 9 meters
                    depth_image[row,col][2] = 255
                    depth_image[row,col][1] = 255-min(255,25*sqrt(distance_squared)) 
            col += 1
            if col >= 1024:
                col = 0
                row += 1
        return depth_image #cv2.flip(cv2.flip( depth_image,0),1)
        
        
    
    ######################
    ## Head Camera Images
    def readHeadCameras(self):
        # Read left and right head cameras
        return self.updateLeftCamera(), self.updateRightCamera()
    ###
    def updateLeftCamera(self):
        # Some functions, particularly those using visual feedback, need quick updates of 
        # the left head camera without the overhead of PCL data, etc.  This function
        # updates only the left camera and returns the image.
        left_data = rospy.wait_for_message("/multisense/camera/left/image_rect_color", Image)
        try: 
            bridge = CvBridge()
            # Convert to OpenCV and invert image (Valkerie's head is upside down)
            left_image = cv2.flip(cv2.flip( bridge.imgmsg_to_cv2(left_data, "bgr8") ,0),1)
            self.left_camera = left_image
            return left_image
        except CvBridgeError as e:
            rospy.loginfo('Error converting ROS Image to OpenCV: ')
            rospy.loginfo(e)  
            return []
    ###
    def updateRightCamera(self):
        # Return OpenCV Images 
        right_data = rospy.wait_for_message("/multisense/camera/right/image_rect_color", Image)
        try: 
            bridge = CvBridge()
            # Convert to OpenCV and invert image (Valkerie's head is upside down)
            right_image = cv2.flip(cv2.flip( bridge.imgmsg_to_cv2(right_data, "bgr8") ,0),1)
            self.right_camera = right_image
            return right_image 
        except CvBridgeError as e:
            rospy.loginfo('Error converting ROS Image to OpenCV: ')
            rospy.loginfo(e)  
            return []
    ###
    def getImage(self):
        return self.left_camera.copy() 

    #######################
    ### Image Processing
            
    ###
    def maskLeftCamera(self,image):
         # Given a numpy image from Valkyrie's LEFT head camera, masks the obscured portion on the right
         ri = 0
         rf = int(len(image)*115/544) 
         ci = int(len(image[0])*956/1024)
         cf = len(image[0])
         for row in xrange(ri,rf):
            for col in xrange(ci+int(.6*(row-ri)),cf):
                image[row][col][0:3] = 0
         ri = int(len(image)*220/544)
         rf = len(image) 
         ci = int(len(image[0])*998/1024)
         cf = len(image[0])
         for row in xrange(ri,rf):
            for col in xrange(cf-int(.1*(row-ri)),cf):
                image[row][col][0:3] = 0
         return image 
     

    ######################
    ### Path image processing 
    def get_mask_path_box(self):
        # Returns a mask of pixels in a starting / goal box (checkered pattern)
        img = self.left_filtered.copy()         # Use the most recent left camera image with background removed 
        ground = self.pcl_threshold_height(-1.7, -1.55)  # Filter out non-ground level pixels

        mask_light = cv2.inRange(img,np.array([115,115,115]),np.array([125,125,125]))  # Light checkered squares
        mask_dark = cv2.inRange(img,np.array([10,10,10]),np.array([18,18,18])) # Dark checkered squares

        img_light = cv2.bitwise_and(ground,ground,mask=mask_light)
        img_dark = cv2.bitwise_and(ground,ground,mask=mask_dark)
        ret, thresh = cv2.threshold(cv2.bitwise_or(img_light, img_dark),9,255,cv2.THRESH_BINARY)

        # Look for big blobs
        ret,mask_threshold = cv2.threshold(thresh.copy(),5,255,cv2.THRESH_BINARY)            # Binary mask 
        kernel = np.ones((3,3),np.float32)/225                            
        mask_blurred = cv2.filter2D(mask_threshold,-1,kernel)                       # Blur
        ret2,mask_threshold2 = cv2.threshold(mask_blurred,2,255,cv2.THRESH_BINARY)  # Threshold again
        edges = cv2.Canny( mask_threshold2, 100,100)
        kernel = np.ones((5,5),np.float32)/225
        edges_blurred = cv2.filter2D(edges,-1,kernel)
        (contours, _) = cv2.findContours( edges_blurred, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        result = np.zeros((544,1024,3), np.uint8)
        good_contour_count = 0
        if len(contours)>0:
            contours.sort(key=lambda x: cv2.arcLength(x,True), reverse=True)
            count = 0
            for c in contours:
                #print cv2.contourArea(c)
                if cv2.contourArea(c) > 400:
                    cv2.drawContours(result, [c], -1, (0,0,255), -1) 
                    good_contour_count += 1
                count += 1
                if count > 5:
                    break 
        return good_contour_count, result 


    def get_mask_path_edge(self):
        # Returns a mask of the edge of the walking path (not the start/goal boxes)
        img = self.left_filtered.copy()         # Use the most recent left camera image with background removed 
        #ground = self.pcl_threshold_height(-1.7, -1.55)  # Filter out non-ground level pixels

        mask_edge = cv2.inRange(img,np.array([62,62,62]),np.array([72,72,72]))  # Look for edge color

        # Look for big blobs
        (contours, _) = cv2.findContours( mask_edge.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        result = np.zeros((544,1024,3), np.uint8)
        good_contour_count = 0
        if len(contours)>0:
            contours.sort(key=lambda x: cv2.arcLength(x,True), reverse=True)
            count = 0
            for c in contours:
                #print cv2.contourArea(c)
                if cv2.contourArea(c) > 2000:
                    cv2.drawContours(result, [c], -1, (0,255,0), -1) 
                    good_contour_count += 1
                count += 1
                if count > 5:
                    break 
        return good_contour_count, result 

    def get_mask_path_path(self):
        # Returns a mask of the path, inside the edges and goal boxes 

        return 



    ###
    def removeBackground(self,img):
        # Assumes non-background items are gray and assumes Mars is not gray
        bg_pixels = img[:,:,0] != img[:,:,1]
        gr_pixels = img[:,:,1] != img[:,:,2]
        img[bg_pixels,:] = 0
        img[gr_pixels,:] = 0
        return img 
    ### Find the wheels of the dish controls
    def findRedWheelPixels(self):
        img = self.left_camera.copy() 
        blue = img[:,:,0] > 10 
        green = img[:,:,1] > 10 
        red = img[:,:,2] < 75
        img[blue,:] = 0
        img[green,:] = 0
        img[red,:] = 0
        return img
    def findBlueWheelPixels(self):
        img = self.left_camera.copy() 
        blue = img[:,:,0] < 100 
        green = img[:,:,1] > 25 
        red = img[:,:,2] > 10
        img[blue,:] = 0
        img[green,:] = 0
        img[red,:] = 0
        return img
    def findDishControlsPixels(self):
        img = self.left_camera.copy()
        blue = img[:,:,0] > 45
        green1 = img[:,:,1] < 55
        green2 = img[:,:,1] > 100
        red1 = img[:,:,2] < 120
        red2 = img[:,:,2] > 160 
        img[blue,:] = 0
        img[green1,:] = 0
        img[green2,:] = 0
        img[red1,:] = 0
        img[red2,:] = 0
        return img



    ###
    def displayImages(self):
        # Utility for displaying stored processed images
        if self.left_camera != None:
            cv2.imshow('left_camera',self.left_camera)
        if self.right_camera != None:
            cv2.imshow('right_camera',self.left_camera)
        #if self.point_cloud != None
        #    cv2.imshow('right_camera',self.left_camera)
        
        # Processed images
        self.path_contour = None
        self.pcl_path = None            # Masked pcl identifying path for walking
        self.pcl_task1_dish = None      # Masked pcl identifing communications dish
        self.pcl_task1_controls = None  # Masked pcl identifing comms dish controls 
        cv2.waitKey(0)
        
    ###
    def highlight(self,base_img,highlight_img):
        img2gray = cv2.cvtColor(highlight_img, cv2.COLOR_BGR2GRAY)
        ret, mask = cv2.threshold(img2gray, 2, 255, cv2.THRESH_BINARY)
        mask_inv = cv2.bitwise_not(mask)
        img1_bg = cv2.bitwise_and(base_img,base_img,mask=mask_inv)
        highlight_img = cv2.addWeighted(base_img,0.3,highlight_img,0.7,0) # Blend base image with highlight
        img2_fg = cv2.bitwise_and(highlight_img,highlight_img,mask=mask)
        base_img = cv2.add(img1_bg,img2_fg)
        return base_img
    
    #######################
    ### Task related processing
    
    ###
    def getBlobLocation(self,img):
        # Given an image of a filled contour, uses the masked pcl to estimate the average distance. 
        # This is useful for creating a map. 
        img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        if cv2.countNonZero(img_gray) < 100:   # TODO: Not sure of the best value here
            return [np.nan, np.nan, np.nan]
        x=[]
        y=[]
        z=[]
        index = 0
        for row in xrange(544):
            for col in xrange(1024):
                if img[row][col][0]>5 or img[row][col][1]>5 or img[row][col][2]>5:   # TODO: use gray image
                    if not isnan( self.point_cloud[1024*row+col][2] ):
                        x.append(self.point_cloud[1024*row+col][0])
                        y.append(self.point_cloud[1024*row+col][1])
                        z.append(self.point_cloud[1024*row+col][2])
        return [sorted(x)[int(len(x)/2)], sorted(y)[int(len(y)/2)], sorted(z)[int(len(z)/2)]]
                
    
    ### Look for communications disk 
    def findDish(self):
        height_img = self.pcl_threshold_height(-0.45, 0.8)
        #cv2.imshow('dish',height_img)
        #cv2.waitKey(0)
        self.pcl_task1_dish = cv2.bitwise_and( height_img, height_img,
                    mask=cv2.inRange(self.left_filtered,np.array([0,0,0]),np.array([200,200,200])))   
                    
        #print "Dish location: ", self.getBlobLocation(self.pcl_task1_dish)
        return self.pcl_task1_dish          
        
    ###
    def findDishControls(self):

        # TODO Use the optimized Opencv functions for this, AND don't need PCL anymore.  

        # Filter out the background 
        left_no_background = self.left_camera.copy() 
        for row in xrange(len(left_no_background)):
            for col in xrange(len(left_no_background[0])):
                point = [int(left_no_background[row][col][0]),int(left_no_background[row][col][1]),int(left_no_background[row][col][2])]
                if (point[0]>40 or abs(point[1]-80)>30 or point[2]<110) and \
                   (point[0]>10 or point[1]>10 or point[2] < 75) and  \
                   (point[0]<100 or point[1]>25 or point[2]>10) :  # Blue wheel pixels 
                    left_no_background[row][col][0]=left_no_background[row][col][1]=left_no_background[row][col][2]=255

        height_img = self.pcl_threshold_height(-1.0, -0.75)       
        controls = cv2.bitwise_and( height_img, height_img,
                    mask=cv2.inRange(left_no_background,np.array([0,0,0]),np.array([200,200,200])))  

        ret,mask_threshold = cv2.threshold(controls,5,255,cv2.THRESH_BINARY)            # Binary mask 
        kernel = np.ones((13,13),np.float32)/225                            
        mask_blurred = cv2.filter2D(mask_threshold,-1,kernel)                       # Blur
        ret2,mask_threshold2 = cv2.threshold(mask_blurred,2,255,cv2.THRESH_BINARY)  # Threshold again
        edges = cv2.Canny( mask_threshold2, 100,100)
        kernel = np.ones((5,5),np.float32)/225
        edges_blurred = cv2.filter2D(edges,-1,kernel)
        (contours, _) = cv2.findContours( edges_blurred, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours)<1:
            return []

        contours.sort(key=lambda x: cv2.arcLength(x,True), reverse=True)
      
        result = np.zeros((544,1024,3), np.uint8)
        cv2.drawContours(result, [contours[0]], -1, (0,255,0), -1)        
        self.pcl_task1_controls = cv2.bitwise_and(controls,result)
        
        #print "Controls location: ", self.getBlobLocation(self.pcl_task1_controls)
        return self.pcl_task1_controls
    

    ###
    def findLeftHand(self):
        # ASSumes the left hand is strictly in the 3rd quadrant of the image
        # ASSumes self.left_camera os up to date
        # TODO 
        img = self.left_camera.copy() 
        img[0:190,:,:] = 0
        img[:,545:,:] = 0
        not_gray1 = img[:,:,0] != img[:,:,1] 
        not_gray2 = img[:,:,1] != img[:,:,2] 
        not_bright = img[:,:,0] < 100
        img[not_gray1,:] = 0
        img[not_gray2,:] = 0
        img[not_bright,:] = 0
        return img
        
        
    ###
    def findRightHand(self):
        # ASSumes the right hand is strictly in the 4th quadrant of the image
        # ASSumes self.left_camera os up to date
        # TODO 
        img = self.left_camera.copy() 
        img[0:190,:,:] = 0
        img[:,:545,:] = 0
        not_gray1 = img[:,:,0] != img[:,:,1] 
        not_gray2 = img[:,:,1] != img[:,:,2] 
        not_bright = img[:,:,0] < 100
        img[not_gray1,:] = 0
        img[not_gray2,:] = 0
        img[not_bright,:] = 0
        return img

    ###
    def getBiggestBlob(self,img):
        # Given an image, returns the minimum circle that surrounds the largest blob in img
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  
        # TODO: should this use blurring??
        _,thresh = cv2.threshold(gray,4,255,0)
        (contours, _) = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key = cv2.contourArea, reverse = True)          
        contours = [cnt for cnt in contours if (cv2.contourArea(cnt)<50000 and cv2.contourArea(cnt)>100)]
        if len(contours) > 0:
            return contours[0]
        return [] 

    ### 
    def getCircle(self,img):
        # Given an image, returns the minimum circle that surrounds the largest blob in img
        gray =  cv2.blur( cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  ,(5,5))
        # TODO: should this use blurring??
        _,thresh = cv2.threshold(gray,4,255,0)
        (contours, _) = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key = cv2.contourArea, reverse = True)          
        contours = [cnt for cnt in contours if (cv2.contourArea(cnt)<50000 and cv2.contourArea(cnt)>100)]
        # Connect the contours
        for i in range(1,len(contours)):
            contours[0] = np.concatenate( (contours[0],contours[i]), axis=0)
        # Minimum radius enclosing circle 
        (x,y),radius = cv2.minEnclosingCircle( contours[0] )
        center = (int(x),int(y))
        radius = int(radius)
        return center, radius 

    ###
    def getRectangle(self,img):
        # Given an image, returns the minimum rectangle that surrounds the largest blob in img
        # Find red contours
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  
        # TODO: should this use blurring??
        _,thresh = cv2.threshold(gray,4,255,0)
        (contours, _) = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key = cv2.contourArea, reverse = True)          
        contours = [cnt for cnt in contours if (cv2.contourArea(cnt)<50000 and cv2.contourArea(cnt)>100)]
        # Minimum radius enclosing circle 
        x,y,w,h = cv2.boundingRect(contours[0])
        #cv2.imshow('contours',contours[0])
        #cv2.waitKey(0)
        #center = (int(x),int(y))
        #dimensions = (int(w),int(h))
        return int(x),int(y),int(w),int(h)

    ###
    def findPath(self):
        # Returns an image with the path identified
        height_img = self.pcl_threshold_height(-1.7, -1.55)
        path = cv2.bitwise_and( height_img, height_img,
                    mask=cv2.inRange(self.left_filtered,np.array([0,0,0]),np.array([200,200,200])))  
     
        ret,mask_threshold = cv2.threshold(path,5,255,cv2.THRESH_BINARY)            # Binary mask 
        kernel = np.ones((9,9),np.float32)/225                            
        mask_blurred = cv2.filter2D(mask_threshold,-1,kernel)                       # Blur
        ret2,mask_threshold2 = cv2.threshold(mask_blurred,2,255,cv2.THRESH_BINARY)  # Threshold again
        edges = cv2.Canny( mask_threshold2, 100,100)
        kernel = np.ones((3,3),np.float32)/225
        edges_blurred = cv2.filter2D(edges,-1,kernel)
        (contours, _) = cv2.findContours( edges_blurred, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours.sort(key=lambda x: cv2.arcLength(x,True), reverse=True)
      
        result = np.zeros((544,1024,3), np.uint8)
        cv2.drawContours(result, [contours[0]], -1, (0,255,0), -1)
        result = cv2.bitwise_and(path,result)
        blue = result[:,:,0].copy()
        green = result[:,:,1].copy()
        result[:,:,0] = green
        result[:,:,1] = blue
        self.pcl_path = result
        
        #print "Path location: ", self.getBlobLocation(self.pcl_path)
        return self.pcl_path

    def getPathImage(self):
        # Returns an image containing BGR values corresponding to Box, Edge, and Path areas
        path = self.findPath() # Start with path and add the other two 
        edge_count, edge = self.get_mask_path_edge()
        box_count, box = self.get_mask_path_box()
        if edge_count > 0:
            # Black out edge contours 
            edge2gray = cv2.cvtColor(edge,cv2.COLOR_BGR2GRAY)
            ret, edge_mask = cv2.threshold(edge2gray, 10, 255, cv2.THRESH_BINARY)
            mask_inv = cv2.bitwise_not(edge_mask)
            path = cv2.bitwise_and(path,path,mask=mask_inv)
            path = cv2.add(path,edge)
        if box_count > 0:
            # Black out box contours 
            box2gray = cv2.cvtColor(box,cv2.COLOR_BGR2GRAY)
            ret, box_mask = cv2.threshold(box2gray, 10, 255, cv2.THRESH_BINARY)
            mask_inv = cv2.bitwise_not(box_mask)
            path = cv2.bitwise_and(path,path,mask=mask_inv)
            path = cv2.add(path,box)
        return path 


        
     
