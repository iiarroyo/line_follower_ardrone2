#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from math import cos, sin

def square():
    pub = rospy.Publisher("cmd_vel",Twist, queue_size=10)
    rospy.init_node('my_control',anonymous=True)
    vel = Twist()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        vel.linear.x = 0.1
        vel.angular.y = 0.1
        x_t=
        pub.publish(vel)
        rate.sleep()

def turn_rigth(vel):
    # regresa rotacion en eje z
    vel.angular.z=1
    vel.linear.x=0


if __name__ == '__main__':
  try:
   square()
  except rospy.ROSInterruptException:
   pass
