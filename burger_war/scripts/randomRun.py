#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random

from geometry_msgs.msg import Twist
#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2


class RandomBot():

    def __init__(self, bot_name):
        # bot name 
        self.name = bot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)






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



        RunCalc(0, 0, 1)        #Sleep
        
        RunCalc(0.5, 90, 1)
        RunCalc(0.5, 0, 1.2)    #1point


        RunCalc(0, 0, 1)        #Sleep
       

if __name__ == '__main__':
    rospy.init_node('random_rulo')
    bot = RandomBot('Random')
    bot.strategy()
