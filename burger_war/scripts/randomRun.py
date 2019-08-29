#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random
import numpy as np

from abstractCcr import *
from geometry_msgs.msg import Twist
#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError


class RandomBot(AbstractCcr):

    # 円を検出する
    def getCircle(self, lower_color, upper_color, min = 25, max = 200):
      MIN_RADIUS = min
      MAX_RADIUS = max

      # HSVによる画像情報に変換
      hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

      # ガウシアンぼかしを適用して、認識精度を上げる
      blur = cv2.GaussianBlur(hsv, (9, 9), 0)

      # 指定した色範囲のみを抽出する
      color = cv2.inRange(blur, lower_color, upper_color)

      # オープニング・クロージングによるノイズ除去
      element8 = np.array([[1, 1, 1], [1, 1, 1], [1, 1, 1]], np.uint8)
      oc = cv2.morphologyEx(color, cv2.MORPH_OPEN, element8)
      oc = cv2.morphologyEx(oc, cv2.MORPH_CLOSE, element8)

      # 輪郭抽出（OpenCVのバージョンによって戻り値の個数が違う）
      img, contours, hierarchy = cv2.findContours(oc, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
      #print("{} contours.".format(len(contours)))

      if len(contours) > 0:
        # 一番大きい指定色領域を指定する
        contours.sort(key=cv2.contourArea, reverse=True)
        cnt = contours[0]

        # 最小外接円を用いて円を検出する
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        center = (int(x), int(y))
        radius = int(radius)

        # 円が小さすぎ、または大きすぎたら円を検出していないとみなす
        if radius < MIN_RADIUS or radius > MAX_RADIUS:
          return None
        else:
          return center, radius
      else:
        return None

    def strategy(self):

        PI = 3.14159265358979
        FPS = 10
        isFindRed = False
        isFindBlue = False
        blueangle = 0

        def SearchCircle():
            if self.isGetLider:
                print(self.scan[0])

            if self.isGetImg:
                # 青円を検出
                getBlueMarker = self.getCircle(np.array([60, 60, 110]), np.array([255, 255, 200])) 
                if getBlueMarker is not None:
                    #getframe[0]:中心座標、getframe[1]:半径
                    print("Blue : {}".format(getBlueMarker[1]))
                    isFindBlue = True
                    blueangle = -((getBlueMarker[0][0] - 320)/100)
                    print(blueangle)
                else:
                    isFindBlue = False

                # ボールを検出
                getRedBall = self.getCircle(np.array([110, 60, 60]), np.array([200, 255, 255])) 
                if getRedBall is not None:
                    #getframe[0]:中心座標、getframe[1]:半径
                    print("Red : {}".format(getRedBall[1]))
                    isFindRed = True
                else:
                    isFindRed = False

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

                #print(twist)

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

        while not rospy.is_shutdown():
            SearchCircle()
       

if __name__ == '__main__':
    rospy.init_node('random_rulo')
    bot = RandomBot(use_camera=True, camera_preview=False, use_lidar=True)
    bot.strategy()
