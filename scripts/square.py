#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from math import sin,cos,pi
import numpy as np

pub = rospy.Publisher("cmd_vel",Twist, queue_size=10)
rospy.init_node('square',anonymous=True)
vel = Twist()
auxRate = 5
rate = rospy.Rate(auxRate)
pos = np.array([[0.0], [0.0]])
# assumptions:
# vel[m/s]

# to do:
# position(): todas las variables deben ser de navdata

def square():
    # seguir cuadrado depues de despegar
    limite1 = np.array([[200.0],[0.0]])
    limiteGrados = (pi/2)
    while (not rospy.is_shutdown() and (go_straight(limite1,pos))):
        pub.publish(vel)
        rate.sleep()
    print("go_straight listo!!")
    rad = [0]
    while (not rospy.is_shutdown()) and (turn_right(limiteGrados,rad)):
        pub.publish(vel)
        rate.sleep()
    print("turn_right listo!!")

def turn_right(gradLim,rad):
    # dar vuelta hasta limite
    vel.angular.z=0
    vel.linear.x=0
    rad[0] += (vel.angular.z * auxRate)
    print(vel)
    if (rad > gradLim):
        return False
    else:
        return True

def go_straight(lim,pos):
    # ir derecho hasta limitemessage
    vel.linear.x = 1
    vel.angular.z = 0
    pos += position(vel)
    if (pos[0] >= lim[0]):
        return False
    else:
        return True

def position(vel):
    # odometria en 2D
    yaw = vel.angular.z
    vx = vel.linear.x
    vy = vel.linear.y
    rotation = np.array([[cos(yaw), -sin(yaw)], [sin(yaw), cos(yaw)]])
    velMat = np.array([[vx], [vy]])
    return (auxRate*np.dot(rotation,velMat))


if __name__ == '__main__':
  try:
   square()
  except rospy.ROSInterruptException:
   pass
