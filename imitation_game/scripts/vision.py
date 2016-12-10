#!/usr/bin/env python

# -*- encoding: UTF-8 -*-

##  @file speech.py
#   @brief File storing information about the speech services.

##  @package speech
#   @brief Package storing information about the speech services.

import sys
import time
import math
import rospy

# OpenCV stuff
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from imitation_game.srv import * #IMPORT YOUR SERVICES
from imitation_game.msg import * #IMPORT YOUR MESSAGES


class image_converter:

  def __init__(self):

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("kinect2/qhd/image_color", Image, self.image_cb)

  def image_cb(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(1)


if __name__ == "__main__":
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")
    cv2.destroyAllWindows()