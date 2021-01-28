#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from math import sin,cos,pi
import numpy as np
'''
to do:
 - position(): todas las variables deben ser de navdata
'''
class Quadrotor:

    def __init__(self):
        self.breakFlag=False
        self.cmdPub = rospy.Publisher("cmd_vel",Twist, queue_size=10)
        self.landPub = rospy.Publisher("ardrone/land", Empty, queue_size=10 )
        self.takeoffPub = rospy.Publisher("ardrone/takeoff", Empty, queue_size=10 )
        rospy.init_node('movement',anonymous=True)
        self.auxRate = 10.0 #Hz
        self.dt = float(1.0/self.auxRate) #ultima vez que se llamo la func(s)
        self.rate = rospy.Rate(self.auxRate)
        self.gradLim = (pi/2.0)#meida vuelta
        self.lim = np.array([[2.0],[0.0]])# solo avanzara 2 unidades(m)

    def callback(self,data):
        # land hasta que reciba mensaje True
        if(data.data):
            self.breakFlag = True

    def listen(self):
        # escuchar status de sqare.py
        rospy.Subscriber("status",Bool,self.callback)

    def land(self):
        # aterrizar

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
          self.landPub.publish(Empty())
          self.rate.sleep()

    def takeoff(self):
        # despegar
        flag = 0
        while (not rospy.is_shutdown()) and flag < 10:
            self.takeoffPub.publish(Empty())
            self.rate.sleep()
            flag+=1


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
        vel.angular.z=1.0
        vel.linear.x=0.0
        # vel.linear.z=0.3
        rad = 0.0

        while (not rospy.is_shutdown()):
            #rotar hasta pi/2(media vueta)
            rad += (vel.angular.z * self.dt)
            if (rad > self.gradLim):
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
        rotation = np.array([[cos(yaw), -sin(yaw)], [sin(yaw), cos(yaw)]])
        velMat = np.array([[vx], [vy]])
        newPos=pos+(self.dt*np.dot(rotation,velMat))
        return newPos

    def end():
        # enviar mensaje status a land.py, para aterrizar
        pub = rospy.Publisher("status", Bool, queue_size=10 )
        while not rospy.is_shutdown():
            pub.publish(True)
            rate.sleep()


if __name__ == '__main__':
  try:
      quad = Quadrotor()
      quad.takeoff()
      quad.square()
      quad.land()
  except rospy.ROSInterruptException:
      pass
