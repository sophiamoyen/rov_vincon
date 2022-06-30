#!/usr/bin/env python
# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from __future__ import print_function
import rospy
import numpy as np
import sys
import cv2
from uuv_control_msgs.srv import GoToIncremental, GoTo
from numpy import pi
from geometry_msgs.msg import Point,Twist,Vector3
from std_msgs.msg import Time
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image

TOL=0.1
TOL_PIX = 5
MAX_VEL = 0.5
MAX_FOWARD_VEL = 0.1

class RovViscon():
    def __init__(self):
        self.rate = rospy.Rate(60)
        self.cmd = Twist()
        self.vel_pub = rospy.Publisher("/rexrov2/cmd_vel", Twist, queue_size = 1)
        self.image_sub = rospy.Subscriber("/rexrov2/rexrov2/camera/camera_image", Image, self.camera_callback)
        self.bridge_object = CvBridge()
        


    def frente(self):
        while not rospy.is_shutdown():
            
            a = Vector3(0, 0, 0)
            self.cmd.angular = a
            
           
            self.vel_pub.publish(self.cmd)
            self.rate.sleep()
    def camera_callback(self,data):
        global cv_image
        cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
        height,width,channels = cv_image.shape
        a,b=  height/2, width/2
        size = height*width

        
        lowerb = np.array([0, 0, 0])
        upperb = np.array([50, 50, 50])

        
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lowerb, upperb)
        pixels = cv2.countNonZero(mask)
        
        cv2.imshow("Mask",mask)
        cv2.waitKey(3)

        M = cv2.moments(mask)

        p = 0.005
        ppixels = 0.0005
        
        if (M['m00'] != 0 ):
            a = int(M['m10']/M['m00'])
            b = int(M['m01']/M['m00'])
            erro_a = (width/2) -  a 
            erro_b = (height/2) - b 
            erro_pixels = size/10 - pixels
            #print(erro_pixels)
            if abs(erro_a) > TOL_PIX:
                self.cmd.linear.y = erro_a * p
            else:
                self.cmd.linear.y = 0

            if abs(erro_b) > TOL_PIX:
                self.cmd.linear.z = erro_b * p
            else:
                self.cmd.linear.z = 0
            if abs(erro_pixels) > 5000:
                self.cmd.linear.x = erro_pixels *p
            else:
                self.cmd.linear.x = 0 

        else:
            print("Estou perdido")
            self.cmd.linear.y = 0
            self.cmd.linear.z = 0
            self.cmd.linear.x = 0

        if self.cmd.linear.z > MAX_VEL:
            self.cmd.linear.z =MAX_VEL
        if self.cmd.linear.z < -MAX_VEL:
            self.cmd.linear.z=-MAX_VEL
        if self.cmd.linear.y > MAX_VEL:
            self.cmd.linear.y =MAX_VEL
        if self.cmd.linear.y < -MAX_VEL:
            self.cmd.linear.y =-MAX_VEL
        if self.cmd.linear.x > MAX_FOWARD_VEL:
            self.cmd.linear.x =MAX_FOWARD_VEL
        if self.cmd.linear.x < -MAX_FOWARD_VEL:
            self.cmd.linear.x =-MAX_FOWARD_VEL





if __name__ == '__main__':
    print('Starting trajetoria')
    rospy.init_node('trajetoria_node')

    if rospy.is_shutdown():
        print('ROS master not running!')
        sys.exit(-1)

    rov = RovViscon()

    rov.frente()

    
    #ponto = Point(30,30,1)
    #serv = rospy.ServiceProxy('/rexrov2/go_to_incremental', GoToIncremental)
    #success = serv(ponto,600, 'lipb')

    #if success:
    #    print('Trajectory successfully generated!')
    #else:
    #    print('Failed')