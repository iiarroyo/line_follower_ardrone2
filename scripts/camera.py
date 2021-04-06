#!/usr/bin/python

# Company : ITESM - Campus Qro
# Author : Israel Ivan Arroyo P A01706190
# Project name : camera.py
# target device: AR parrot drone 2, 2.3.3 frimware version
# target simulation: gazebo v9.0.0 with tum_simulator_meodic ros library
# tool version : python 2.7.17
# Description: Make drone follow a green line

import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata

from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError

import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from math import sin,cos,pi

class Quadrotor:

    def __init__(self):
        self.cmdPub = rospy.Publisher("cmd_vel",Twist, queue_size=10)
        self.takeoffPub = rospy.Publisher("ardrone/takeoff", Empty, queue_size=10)
        self.navdataSub = rospy.Subscriber("ardrone/navdata",Navdata,self.callback)
        self.imgSub = rospy.Subscriber("ardrone/bottom/image_raw",Image,self.imgCallback)
        self.auxRate = 10.0 #Hz
        self.lim = np.array([[3.0],[0.0]])# solo avanzara 1 unidades(m)
        self.dt = float(1.0/self.auxRate) #ultima vez que se llamo la func(s)
        self.height = 640
        self.width = 360
        self.navdataVar = Navdata()

        self.mask = np.ndarray(shape=(640,360))
        self.rate = rospy.Rate(10)
        self.mColor = 0

    def callback(self,data):
        # land hasta que reciba mensaje True
        self.navdataVar = data

    def imgCallback(self,data):
        # returns numpy.ndarray format Image
        self.height = 640
        self.width = 360
        image_message = data
        bridge = CvBridge()
        try:
            image =  bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        upper_green_hsv =np.array([70,255,255])
        lower_green_hsv =np.array([50,235,235])
        self.mask = cv2.inRange(hsv,lower_green_hsv,upper_green_hsv)

    def liveVideo(self):
        aux = 0
        bridge = CvBridge()
        while True:
            print(self.imageView[319,179])
            # imgplot = plt.imshow(self.imageView)
            # plt.show()
            #cv2.imshow(cv_image)
            aux += 1
            self.rate.sleep()

    def takeoff(self):
        # despegar
        while (not rospy.is_shutdown()) and (self.navdataVar.state != 3):
            # print(self.navdataVar.altd)
            self.takeoffPub.publish(Empty())
            self.rate.sleep()

    def go_straight(self,pos= np.array([[0.0], [0.0]])):
        # ir derecho hasta limite
        vel = Twist()
        vel.linear.x = 1.0
        vel.angular.z = 0.0
        # vel.linear.z=0.3

        while ((not rospy.is_shutdown() )and
         (pos[0] <= self.lim[0] and pos[1]<=self.lim[1])):
         # mientras no se llegue a las coordenadas lim
            pos = self.position(vel,pos)
            self.cmdPub.publish(vel)
            self.rate.sleep()

    def stop(self):
        # ir derecho hasta limite
        vel = Twist()
        vel.linear.x = 0.0
        vel.linear.y = 0.0
        vel.linear.z = 0.2
        vel.angular.z = 0.0
        auxVel = 1
        while ((not rospy.is_shutdown()) and (auxVel > 0)):
            # mientras no se llegue a las coordenadas lim
            self.cmdPub.publish(vel)
            auxVel = self.navdataVar.vx + self.navdataVar.vy
            self.rate.sleep()

    def hover(self):
        # ir derecho hasta limite
        while (not rospy.is_shutdown()):
            m = cv2.moments(self.mask,False)
            try:
                cx,cy = m['m10']/m['m00'],m['m01']/m['m00']
            except ZeroDivisionError:
                cy,cx = (self.height)/2,(self.width)/2

            error_x = cx - (self.width)/2
            vel = Twist()
            vel.linear.x = 0.2
            vel.angular.z = -error_x/100
            self.cmdPub.publish(vel)
            self.rate.sleep()

    def position(self,vel,pos):
        # odometria en 2D
        yaw = vel.angular.z
        vx = vel.linear.x
        vy = vel.linear.y
        # yaw = self.navdataVar.rotZ
        # vx = self.navdataVar.vx
        # vy = self.navdataVar.vy
        rotation = np.array([[cos(yaw), -sin(yaw)], [sin(yaw), cos(yaw)]])
        velMat = np.array([[vx], [vy]])
        newPos=pos+(self.dt*np.dot(rotation,velMat))
        return newPos

if __name__ == '__main__':
  try:
      print("start")
      rospy.init_node('movement',anonymous=True)
      quad = Quadrotor()
      quad.takeoff()
      try:
          quad.hover()
      except KeyboardInterrupt:
          quad.stop()
      # quad.go_straight()
      # quad.stop()
      #print("hover")

      #print("end")
  except rospy.ROSInterruptException:
      pass
