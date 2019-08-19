#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random

from abstractCcr import *
from geometry_msgs.msg import Twist
#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2


class RandomBot(AbstractCcr):

    def strategy(self):

        PI = 3.14159265358979
        FPS = 10

        #speed: -0.5~0.5
        #angle: -180~180
        #time : second
        def RunCalc(speed, angle, time):
            
            cnt = 0
            r = rospy.Rate(FPS) # change speed
            time = time * FPS   # dhange to second

            twist = Twist()

            twist.linear.x = speed
            twist.angular.z = PI * angle / 180

            self.vel_pub.publish(twist)

            while cnt < time:

                print(twist)

                cnt = cnt + 1
                r.sleep()
        
        def GoNext():
            RunCalc(0, 46, 1)
            RunCalc(0, 0, 0.5)
            RunCalc(0.25, 0, 2.9)
            RunCalc(0, 0, 0.5)
            RunCalc(0, -45, 1)
            RunCalc(0, 0, 0.5)
        
            RunCalc(0, -90, 1)
            RunCalc(0, 0, 0.5)
        
            RunCalc(0, -90, 1)
            RunCalc(0, 0, 0.5)
            RunCalc(0, 90, 1)
            RunCalc(0, 0, 0.5)

            RunCalc(0, 44, 1)
            RunCalc(0, 0, 0.5)
            RunCalc(0.25, 0.5, 2.9)
            RunCalc(0, 0, 0.5)
            RunCalc(0, -135, 1)
            RunCalc(0, 0, 1)


        RunCalc(0, 0, 1)
        RunCalc(0.25, -0.5, 3.1) #1Point
        RunCalc(0, 0, 0.5)

	while not rospy.is_shutdown():
            GoNext()
       

if __name__ == '__main__':
    rospy.init_node('random_rulo')
    bot = RandomBot()
    bot.strategy()
