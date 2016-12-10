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

from imitation_game.srv import * #IMPORT YOUR SERVICES
from imitation_game.msg import * #IMPORT YOUR MESSAGES


# ## @brief Function making the NAO speak.
# # @param   text             words that NAO should say
# # @return True if the action succeeded.
# #
# def FUNCTION_NAME(req):
#     resp = SERVICE_TYPEResponse()
#     resp.firstThingToReturn = 1
#     resp.secondThingToReturn = 2
#     return resp

# ## @brief Service server for the speech service.
# #
# def FUNCTION_NAME_server():
#     s = rospy.Service("SERVICE_NAME", SERVICE_TYPE, FUNCTION_NAME)

# ## @brief Function making the NAO speak.
# # @param   text             words that NAO should say
# # @return True if the action succeeded.
# #
# def FUNCTION_NAME(req):
#     resp = SERVICE_TYPEResponse()
#     resp.firstThingToReturn = 1
#     resp.secondThingToReturn = 2
#     return resp

# ## @brief Service client for the speech service.
# #
# def FUNCTION_NAME_client(par1,par2):
#     rospy.wait_for_service('SERVICE_NAME')
#     try:
#         name = rospy.ServiceProxy('SERVICE_NAME', SERVICE_TYPE)
#         resp1 = name(input1 = par1, input2 = par2) 
#         #or send them without keywords in order specified in .srv file
#         resp1 = name(par1,par2)

#     except rospy.ServiceException, e:
#         print "Service call failed: %s"%e

# def PUBLISHER():
#     pub = rospy.Publisher('NAME OF THE TOPIC', MSG_TYPE)
#     r = rospy.Rate(10) #10hz
#     msg = MSG_TYPE()
#     msg.name = "ROS User"
#     msg.age = 4

#     # PUBLISHING CONSTANTLY
#     while not rospy.is_shutdown(): 
#         rospy.loginfo(msg) #COUT FOR ROS
#         pub.publish(msg)
#     # PUBLISHING ONCE 
#     rospy.loginfo(msg) #COUT FOR ROS
#     pub.publish(msg)

# def CALLBACK_NAME(object):
#     # DO SOME STAFF
#     # TO ACCESS DATA USE THEIR NAMES object.thing1
#     return object

## @brief Function making the NAO speak.
# @param   text             words that NAO should say
# @return True if the action succeeded.
#
def blob_service(req):
    resp = blob_centerResponse()
    resp.x = 1
    resp.y = 2
    resp.valid = True
    resp.color = 'blue'
    return resp

def blob_service_server():
    s = rospy.Service("blob_center_srv", blob_center, blob_service)

## @brief Service client for the speech service.
#
def blob_service_client():
    rospy.wait_for_service('blob_center_srv')
    try:
        name = rospy.ServiceProxy('blob_center_srv', blob_center)
        resp1 = name()
        rospy.loginfo("Service response:")
        rospy.loginfo(resp1.x)
        rospy.loginfo(resp1.y)
        rospy.loginfo(resp1.color)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def blob_center_msg():
    pub = rospy.Publisher('blob_center_msg', BlobCenter, queue_size=2)
    #r = rospy.Rate(10) #10hz
    msg = BlobCenter()
    msg.color = "red"
    msg.x = 4
    msg.y = 5
    msg.valid = True
    # PUBLISHING CONSTANTLY
    while not rospy.is_shutdown(): 
        rospy.loginfo("Publishing") #COUT FOR ROS
        pub.publish(msg)

def blob_callback(object):
    # DO SOME STAFF
    # TO ACCESS DATA USE THEIR NAMES object.thing1
    rospy.loginfo(object.x)
    rospy.loginfo(object.y)
    rospy.loginfo(object.color)
    return object

## @brief Initialization function of the node.
#
# Runs all servers:
# - say aloud
#
def main():
    rospy.init_node('ig_speech') #NAME OF THE NODE TO RUN
    # advertizing service
    #blob_service_server()
    #blob_service_client()
    blob_center_msg()
    # subscribing to the topic
    # rospy.Subscriber("blob_center_msg", BlobCenter, blob_callback)
    # DO SOME STAFF
    rospy.spin()

if __name__ == "__main__":
    main()
