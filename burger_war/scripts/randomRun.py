#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random
import numpy as np
import time

from abstractCcr import *
from geometry_msgs.msg import Twist
from enum import Enum
#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError

class ActMode(Enum):
    INIT = 0
    NORMAL = 1
    WALL = 2
    CHASE = 3
    BERSERK = 4

class RandomBot(AbstractCcr):
    isFindGreen = False
    isFindBlue = False
    isFindRed = False
    blueangle = 0
    greenangle = 0
    redangle = 0
    isWallDetect = False
    goNext = False
    actMode = ActMode.INIT

    # 円を検出する
    def getCircle(self, lower_color, upper_color, min = 25, max = 300):
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

    # ボールを検出する
    def getBall(self, min = 25, max = 300):
      MIN_RADIUS = min
      MAX_RADIUS = max

      # HSVによる画像情報に変換
      hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

      # ガウシアンぼかしを適用して、認識精度を上げる
      blur = cv2.GaussianBlur(hsv, (9, 9), 0)

      # 指定した色範囲のみを抽出する
      color1 = cv2.inRange(blur, np.array([0, 64, 0]), np.array([30, 255, 255]))
      color2 = cv2.inRange(blur, np.array([150, 64, 0]), np.array([179, 255, 255]))
      color = color1 + color2

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
        r = rospy.Rate(FPS)
        actMode = ActMode.INIT
        t1 = time.time()

        def GetDistance():
            prevalue = self.isWallDetect
            if self.isGetLidar:
                if(self.scan.ranges[0] < 0.30 or self.scan.ranges[4] < 0.30 or self.scan.ranges[355] < 0.30):
                    self.isWallDetect = True
                elif(self.scan.ranges[0] > 0.35 and self.scan.ranges[4] > 0.35 and self.scan.ranges[355] > 0.35):
                    self.isWallDetect = False
                    if prevalue is not self.isWallDetect:
                        print("Next")
                        self.goNext = True

        def FindSpace():
            if self.isGetLidar:
                print(self.scan.ranges[0])
                if(self.scan.ranges[0] > 0.60 and self.scan.ranges[4] > 0.60 and self.scan.ranges[355] > 0.60):
                    return True
                else:
                    GreenMarker = self.getCircle(np.array([40, 75, 75]), np.array([80, 255, 255]))
                    if GreenMarker is not None:
                        self.isFindGreen = True
                        self.greenangle = -((GreenMarker[0][0] - 320)/10)
                        self.actMode = ActMode.CHASE
                        print(self.actMode)
                        return True
                    else:
                        return False
            else:
                return False

        def SearchCircle():
            if self.isGetImg:
                # 青円を検出
                BlueMarker = self.getCircle(np.array([100, 75, 75]), np.array([140, 255, 255])) 
                if BlueMarker is not None:
                    #getframe[0]:中心座標、getframe[1]:半径
                    print("Blue : {}".format(BlueMarker[1]))
                    self.isFindBlue = True
                    self.blueangle = -((BlueMarker[0][0] - 320)/10)
                    print(self.blueangle)
                else:
                    self.isFindBlue = False
                    self.blueangle = 0

                # 緑円を検出
                GreenMarker = self.getCircle(np.array([40, 75, 75]), np.array([80, 255, 255])) 
                if GreenMarker is not None:
                    #getframe[0]:中心座標、getframe[1]:半径
                    print("Green : {}".format(GreenMarker[1]))
                    self.isFindGreen = True
                    self.greenangle = -((GreenMarker[0][0] - 320)/10)
                else:
                    self.isFindGreen = False
            r.sleep()

        def SearchGreen():
            if self.isGetImg:
                # 緑円を検出
                GreenMarker = self.getCircle(np.array([40, 75, 75]), np.array([80, 255, 255])) 
                if GreenMarker is not None:
                    #getframe[0]:中心座標、getframe[1]:半径
                    print("Green : {}".format(GreenMarker[1]))
                    self.isFindGreen = True
                    self.greenangle = -((GreenMarker[0][0] - 320)/10)
                else:
                    self.isFindGreen = False
                    self.greenangle = 0

        def SearchRed():
            if self.isGetImg:
                # 赤を検出
                red = self.getBall(max=160) 
                if red is not None:
                    #getframe[0]:中心座標、getframe[1]:半径
                    print("Red : {}".format(red[1]))
                    self.isFindRed = True
                    self.redangle = -((red[0][0] - 320)/10)
                    #print(self.redangle)
                    return True
                else:
                    self.isFindRed = False
                    self.redangle = 0
                    return False
            else:
                return False

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

        def Move(speed,angle):
            twist = Twist()
            twist.linear.x = speed
            twist.angular.z = PI * angle / 180

            self.vel_pub.publish(twist)
            r.sleep()
        

        RunCalc(0, 0, 1)
        RunCalc(0.25, -0.5, 3.3) #1Point
        RunCalc(0, 0, 0.5)
        spin = 40
        backcnt = 0

        while not rospy.is_shutdown():
            elapsedTime = time.time() - t1
            if elapsedTime > 40.0:
                self.actMode = ActMode.BERSERK
                print(self.actMode)

            angle = 0
            
            GetDistance()
            SearchCircle()

            if self.isWallDetect:
                #RunCalc(0, 0, 0.5)
                Move(-0.15,0)
                backcnt = backcnt + 1
                print(backcnt)
                if(backcnt > 100):
                    self.actMode = ActMode.BERSERK
                    print(self.actMode)   
            else:
                backcnt = 0
                if self.isFindBlue:
                    angle = self.blueangle
                    nomarker = 0
                else:
                    nomarker = nomarker + 1
                if self.isFindGreen:
                    print("Green")
                    angle = self.greenangle
                    nomarker = 0
                    self.actMode = ActMode.CHASE
                    print(self.actMode)

                Move(0.25,angle)
            
            if self.goNext:
                RunCalc(0, 0, 0.5)
                start = time.time()
                while not FindSpace():
                    Move(0,spin)
                    #RunCalc(0,spin,0.2)
                    print("Spin : {}".format(spin))
                    timeout = time.time() - start
                    if(timeout > 10):
                        self.actMode = ActMode.BERSERK
                        print(self.actMode)
                        break
                spin = spin * -1
                self.goNext = False

            if self.actMode is ActMode.CHASE:
                while not self.isFindGreen:
                    SearchGreen()
                    Move(0.5,self.greenangle)
                self.actMode = ActMode.NORMAL
            
            if self.actMode is ActMode.BERSERK:
                RunCalc(0, 0, 0.5)
                while not rospy.is_shutdown():
                    SearchGreen()
                    cnt = 0
                    while not self.isFindGreen:
                        SearchGreen()
                        Move(0,90)
                        cnt = cnt + 1
                        if(cnt > 40):
                            while not FindSpace():
                                Move(0,-90)
                            RunCalc(0.5, 0, 2.5)
                            break
                    GetDistance()
                    if self.isWallDetect:
                        Move(-0.15,0)
                    else:
                        angle = 0
                        if self.isFindGreen:
                            angle = self.greenangle
                        Move(0.5,angle)
       

if __name__ == '__main__':
    rospy.init_node('random_rulo')
    bot = RandomBot(use_camera=True, camera_preview=False, use_lidar=True)
    bot.strategy()
