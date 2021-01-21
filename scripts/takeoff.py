#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

def takeoff():
  pub = rospy.Publisher("ardrone/takeoff", Empty, queue_size=10 )
  rospy.init_node('takeoff', anonymous=True)
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    pub.publish(Empty())
    rate.sleep()

def square():
    pub = rospy.Publisher("cmd_vel",Twist, queue_size=10)
    rospy.init_node('my_control',anonymous=True)
    vel = Twist()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        vel.linear.x = 0.1
        pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
  try:
   takeoff()
  except rospy.ROSInterruptException:
   pass
