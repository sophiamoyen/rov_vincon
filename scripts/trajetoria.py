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
from uuv_control_msgs.srv import GoToIncremental, GoTo, InitWaypointsFromFile
from numpy import pi
from geometry_msgs.msg import Point,Twist,Vector3
from std_msgs.msg import Time
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String, Time


class RovTraject():
    def __init__(self):
        self.rate = rospy.Rate(60)
        

    def trajetoria_unitaria(self,x,y,z):
        try:
            rospy.wait_for_service('/rexrov2/go_to_incremental', timeout=30)
        except rospy.ROSException:
            raise rospy.ROSException('Service not available! Closing node...')
        service_sub = rospy.ServiceProxy('/rexrov2/go_to_incremental',GoToIncremental)
        ponto = Point(x,y,z)
        success = service_sub(ponto,0.3,'cubic')
        if success:
            print("Trajetoria unitaria gerada")
        else:
            print("Erro na geracao da trajetoria")


    def trajetoria_file(self):
        filename = rospy.get_param('~filename')
        interpolator = rospy.get_param('~interpolator')
        start_now = True
        print(filename)
        start_time = rospy.Time.now().to_sec()
        try:
            rospy.wait_for_service('/rexrov2/init_waypoints_from_file', timeout=30)
        except rospy.ROSException:
            raise rospy.ROSException('Service not available! Closing node...')
        
        init_wp = rospy.ServiceProxy('/rexrov2/init_waypoints_from_file',InitWaypointsFromFile)
        success = init_wp(Time(rospy.Time.from_sec(start_time)),start_now, String(filename),interpolator)
        if success:
            print("Trajetoria gerada")
        else:
            print("Erro na geracao da trajetoria")
    





if __name__ == '__main__':
    print('Starting trajetoria')
    rospy.init_node('trajetoria_node')

    if rospy.is_shutdown():
        print('ROS master not running!')
        sys.exit(-1)

    rov = RovTraject()

    rov.trajetoria_unitaria(0,0,-20)

    
    #ponto = Point(30,30,1)
    #serv = rospy.ServiceProxy('/rexrov2/go_to_incremental', GoToIncremental)
    #success = serv(ponto,600, 'lipb')

    #if success:
    #    print('Trajectory successfully generated!')
    #else:
    #    print('Failed')