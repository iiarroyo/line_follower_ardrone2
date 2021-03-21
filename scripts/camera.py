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

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

class Quadrotor:

    def __init__(self):


        self.imgSub = rospy.Subscriber("ardrone/bottom/image_raw",Image,self.imgCallback)

        rospy.init_node('movement',anonymous=True)
        self.imageView = np.ndarray(shape=(640,360))

    def imgCallback(self,data):
        # returns numpy.ndarray format Image
        image_message = data
        bridge = CvBridge()
        self.imageView = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')

    def liveVideo(self):
        aux = 0
        bridge = CvBridge()
        while aux < 5:
            #print(self.imageView[319,179])
            imgplot = plt.imshow(self.imageView)
            plt.show()
            #cv2.imshow(cv_image)
            aux += 1

if __name__ == '__main__':
  try:
      quad = Quadrotor()
      quad.liveVideo()
  except rospy.ROSInterruptException:
      pass
