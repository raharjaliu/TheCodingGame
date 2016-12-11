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

from gtts import gTTS
import os

from guessing_game.srv import * #IMPORT YOUR SERVICES


## @brief Function handling the speech.
# @param   text             words that should be said
# @return True if the action succeeded.
#
def system_speech(req):
    tts = gTTS(text=req.text, lang='en')
    tts.save("text.mp3")
    os.system("mpg321 text.mp3")
    return text_to_speechResponse(True)

def system_speech_server():
    s = rospy.Service("text_to_speech", text_to_speech, system_speech)

## @brief Initialization function of the node.
#
if __name__ == "__main__":
    rospy.init_node('gg_speech')
    # Advertizing services
    system_speech_server()
    rospy.loginfo("Registered service gg_system_speech.")
    # DO SOME STAFF
    rospy.spin()