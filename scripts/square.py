#!/usr/bin/python

# Company : ITESM - Campus Qro
# Author : Israel Ivan Arroyo P A01706190
# Project name : square.py
# target device: AR parrot drone 2, 2.3.3 frimware version
# target simulation: gazebo v9.0.0 with tum_simulator_meodic ros library
# tool version : python 2.7.17
# Description: Make drone follow a square pattern.

import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata
from math import sin,cos,pi
import numpy as np

class Quadrotor:

    def __init__(self):
        self.cmdPub = rospy.Publisher("cmd_vel",Twist, queue_size=10)
        self.landPub = rospy.Publisher("ardrone/land", Empty, queue_size=10 )
        self.takeoffPub = rospy.Publisher("ardrone/takeoff", Empty, queue_size=10)

        self.navdataSub = rospy.Subscriber("ardrone/navdata",Navdata,self.callback)
        rospy.init_node('square',anonymous=True)
        self.auxRate = 10.0 #Hz
        self.dt = float(1.0/self.auxRate) #ultima vez que se llamo la func(s)
        self.rate = rospy.Rate(self.auxRate)
        self.gradLim = (pi/2.0)#meida vuelta
        self.lim = np.array([[1.0],[0.0]])# solo avanzara 1 unidades(m)
        self.navdataVar = Navdata()

    def callback(self,data):
        # land hasta que reciba mensaje True
        self.navdataVar = data

    def land(self):
        # aterrizar

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
          self.landPub.publish(Empty())
          self.rate.sleep()

    def takeoff(self):
        # despegar
        while (not rospy.is_shutdown()):# and (self.navdataVar.state != 3):
            # print(self.navdataVar.altd)
            self.takeoffPub.publish(Empty())
            self.rate.sleep()


    def square(self):
        # seguir cuadrado depues de despegar
        pos = np.array([[0.0], [0.0]])
        self.go_straight(pos)
        self.turn_right()
        self.go_straight(pos)
        self.turn_right()
        self.go_straight(pos)
        self.turn_right()
        self.go_straight(pos)
        self.turn_right()

    def turn_right(self):
        # dar vuelta hasta limite
        vel = Twist()
        vel.angular.z=-1.0
        vel.linear.x=0.0
        rad = 0.0

        while (not rospy.is_shutdown()):
            #rotar hasta pi/2(media vueta)
            rad += (vel.angular.z * self.dt)
            if (abs(rad) > self.gradLim):
                return
            else:
                self.cmdPub.publish(vel)
            self.rate.sleep()


    def go_straight(self,pos):
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
      quad = Quadrotor()
      quad.takeoff()
      quad.square()
      quad.land()
  except rospy.ROSInterruptException:
      pass
# Reference for navdata type: https://ardrone-autonomy.readthedocs.io/en/latest/reading.html
