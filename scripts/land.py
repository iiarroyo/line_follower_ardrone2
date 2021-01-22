#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Empty

# to do:
# agregar y crear topico, para determinar si se debe aterrizar
class Quadrotor:

    def __init__(self):
        self.breakFlag=False

    def callback(self,data):
        # land hasta que reciba mensaje True
        if(data.data):
            self.breakFlag = True

    def listen(self):
        # escuchar status de sqare.py
        rospy.Subscriber("status",Bool,self.callback)

    def land(self):
        # aterrizar
        pub = rospy.Publisher("ardrone/land", Empty, queue_size=10 )
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
          pub.publish(Empty())
          rate.sleep()

    def takeoff(self):
        # despegar
        pub = rospy.Publisher("ardrone/takeoff", Empty, queue_size=10 )
        rospy.init_node("land",anonymous=True)
        rate = rospy.Rate(10) # 10hz

        while (not rospy.is_shutdown())and not self.breakFlag:
            pub.publish(Empty())
            self.listen()
            rate.sleep()

if __name__ == '__main__':
  try:
      quad = Quadrotor()
      quad.takeoff()
      quad.land()
  except rospy.ROSInterruptException:
      pass
