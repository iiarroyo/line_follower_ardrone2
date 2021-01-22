#!/usr/bin/env python
import rospy
from takeoff import *
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from math import sin,cos,pi
import numpy as np

pub = rospy.Publisher("cmd_vel",Twist, queue_size=10)
rospy.init_node('square',anonymous=True)
auxRate = 10.0 #Hz
dt = float(1.0/auxRate) #segundos
rate = rospy.Rate(auxRate)
gradLim = (pi/2.0)
lim = np.array([[2.0],[0.0]])
# assumptions:
# vel[m/s]

# to do:
# position(): todas las variables deben ser de navdata

def square():
    # seguir cuadrado depues de despegar
    pos = np.array([[0.0], [0.0]])
    go_straight(pos)
    turn_right()
    go_straight(pos)
    turn_right()
    go_straight(pos)
    turn_right()
    go_straight(pos)
    turn_right()

def turn_right():
    # dar vuelta hasta limite
    vel = Twist()
    vel.angular.z=1.0
    vel.linear.x=0.0
    rad = 0.0

    while (not rospy.is_shutdown()):
        rad += (vel.angular.z * dt)
        if (rad > gradLim):
            return
        else:
            pub.publish(vel)
        rate.sleep()

def go_straight(pos):
    # ir derecho hasta limite
    vel = Twist()
    vel.linear.x = 1.0
    vel.angular.z = 0.0

    while ((not rospy.is_shutdown() )and
     (pos[0] <= lim[0] and pos[1]<=lim[1])):
     # mientras no se llegue a las coordenadas lim
        pos = position(vel,pos)
        pub.publish(vel)
        rate.sleep()

def position(vel,pos):
    # odometria en 2D
    yaw = vel.angular.z
    vx = vel.linear.x
    vy = vel.linear.y
    rotation = np.array([[cos(yaw), -sin(yaw)], [sin(yaw), cos(yaw)]])
    velMat = np.array([[vx], [vy]])
    newPos=pos+(dt*np.dot(rotation,velMat))
    return newPos

def end():
    # enviar mensaje status a land.py, para aterrizar
    pub = rospy.Publisher("status", Bool, queue_size=10 )
    while not rospy.is_shutdown():
        pub.publish(True)
        rate.sleep()


if __name__ == '__main__':
    try:
        square()
        end()
    except rospy.ROSInterruptException:
        pass
