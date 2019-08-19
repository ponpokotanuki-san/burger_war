#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from abc import ABCMeta, abstractmethod
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import cv2


class AbstractCcr(object):
    __metaclass__ = ABCMeta
    def __init__(self, 
                 use_lidar=False ,use_camera=False,
                 use_opt=False, use_usonic=False,use_odometry=False,
                 camera_preview=False):

        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

        # lidar scan subscriber
        if use_lidar:
            self.scan = LaserScan()
            self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)

        # optical sensor scan subscriber
        if use_opt:
            self.opt = [LaserScan(), LaserScan()]
            self.opt_left_sub = rospy.Subscriber('opt_left', LaserScan, self.optLeftCallback)
            self.opt_right_sub = rospy.Subscriber('opt_right', LaserScan, self.optRightCallback)

        # ultrasonic sensor scan subscriber
        if use_usonic:
            self.usonic = [LaserScan(), LaserScan()]
            self.usonic_left_sub = rospy.Subscriber('us_left', LaserScan, self.usonicLeftCallback)
            self.usonic_right_sub = rospy.Subscriber('us_right', LaserScan, self.usonicRightCallback)

        # odometry subscriber
	if use_odometry:
	    self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCallback)

        # camera subscribver
        # please uncoment out if you use camera
        if use_camera:
            # for convert image topic to opencv obj
            self.img = None
            self.camera_preview = camera_preview
            self.bridge = CvBridge()
            self.image_sub = rospy.Subscriber('camera/image_raw', Image, self.imageCallback)

    # lidar scan topic call back sample
    # update lidar scan state
    def lidarCallback(self, data):
        self.scan = data

    # optical  scan topic call back sample
    # update lidar scan state
    def optLeftCallback(self, data):
        self.opt[0] = data

    def optRightCallback(self, data):
        self.opt[1] = data

    # usonic  scan topic call back sample
    # update lidar scan state
    def usonicLeftCallback(self, data):
        self.usonic[0] = data

    def usonicRightCallback(self, data):
        self.usonic[1] = data
    
    # odometry call back
    def odomCallback(self, data):
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.camera_preview:
          cv2.imshow("Image window", self.img)
          cv2.waitKey(1)

    @abstractmethod
    def strategy(self):
        pass

