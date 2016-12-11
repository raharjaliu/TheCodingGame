#!/usr/bin/env python

# -*- encoding: UTF-8 -*-

##  @file speech.py
#   @brief File storing information about the speech services.

##  @package speech
#   @brief Package storing information about the speech services.

import sys
import os
import re
import subprocess
import time
import math
import rospy

# OpenCV stuff
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from imitation_game.srv import * #IMPORT YOUR SERVICES
from imitation_game.msg import * #IMPORT YOUR MESSAGES

# path_to_darknet = '/Users/rootmac/Documents/workspace/darknet/'

class main_node:

  def __init__(self):

    self.bridge = CvBridge()
    self.vision_sub = rospy.Subscriber("image_converter", Image, self.image_cb)
    self.speech_sub = rospy.Subscriber("gg_speech", Image, self.image_cb)

  def predict_label(self,path):

    # change working dir and run darknet
    # in current version, darknet has to be installed in home directory (~/darknet/)
    # call the bash script install_darknet.sh to install darknet
    os.chdir(os.path.join(os.path.expanduser("~"), "darknet/"))
    command = './darknet classifier predict cfg/imagenet22k.dataset cfg/extraction.cfg extraction.weights ' + path
    val = subprocess.check_output(command.split(' '))

    # postprocess the results and return
    val = val.split('\n')
    ret = [d for d in val if re.search('[a-z]+: [\d.]+', d)]

    return {d.split(': ')[0] : float(d.split(': ')[1]) for d in ret}

  def image_cb(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv2.imwrite("in.png", cv_image)
    cv2.imshow("Image window", cv_image)
    rospy.loginfo(self.predict_label("in.png"))

    cv2.waitKey(1)


if __name__ == "__main__":
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")
    cv2.destroyAllWindows()