#!/usr/bin/python

# Company : ITESM - Campus Qro
# Author : Israel Ivan Arroyo P A01706190
# Project name : movement.py
# target device: AR parrot drone 2, 2.3.3 frimware version
# target simulation: gazebo v9.0.0 with tum_simulator_meodic ros library
# tool version : python 2.7.17
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata
from math import sin,cos,pi
import numpy as np

class Quadrotor:

    def __init__(self):
        self.breakFlag=False
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
        while (not rospy.is_shutdown()) and (self.navdataVar.altd < 800):
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
        vel.angular.z=-0.5
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
        vel.linear.x = 0.1
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
# Reference : https://ardrone-autonomy.readthedocs.io/en/latest/reading.html
# header:
#   seq: 4785
#   stamp:
#     secs: 23
#     nsecs: 930000000
#   frame_id: "ardrone_base_link"
# batteryPercent: 99.1004180908
# state: 3
# magX: 0
# magY: 0
# magZ: 0
# pressure: 0
# temp: 0
# wind_speed: 0.0
# wind_angle: 0.0
# wind_comp_angle: 0.0
# rotX: 0.0520852431655
# rotY: -0.0970923304558
# rotZ: -0.670082449913
# altd: 599
# vx: 951.08013916
# vy: 3.64785599709
# vz: 11.7337741852
# ax: -0.00248119980097
# ay: 0.000215476015001
# az: 1.00660967827
# motor1: 0
# motor2: 0
# motor3: 0
# motor4: 0
# tags_count: 0
# tags_type: []
# tags_xc: []
# tags_yc: []
# tags_width: []
# tags_height: []
# tags_orientation: []
# tags_distance: []
# tm: 23930000.0
