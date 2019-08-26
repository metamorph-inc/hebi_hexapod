#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
# Description:
#   Subscribe to /keystroke ROS Topic (String)
#   Publish to /cmd_vel Topic (Twist)
"""

import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist


key_mapping = {'q': [(0,0,0),(0,0,1)], 'w': [(1,0,0),(0,0,0)], 'e': [(0,0,0),(0,0,-1)],
               'a': [(0,1,0),(0,0,0)], 's': [(0,0,0),(0,0,0)], 'd': [(0,-1,0),(0,0,0)],
               'x': [(-1,0,0),(0,0,0)]}
g_target_twist = None


def send_twist(twist_pub):
    if g_target_twist is not None:
        # TODO: Add velocity ramp
        twist_pub.publish(g_target_twist)


def keystroke_cb(msg):
    global g_target_twist
    if len(msg.data) != 0 and msg.data[0] in key_mapping:
        key = msg.data[0]
        rospy.loginfo("Received msg: %s", key)
        g_target_twist = Twist()
        g_target_twist.linear.x = key_mapping[key][0][0]
        g_target_twist.linear.y = key_mapping[key][0][1]
        g_target_twist.linear.z = key_mapping[key][0][2]
        g_target_twist.angular.x = key_mapping[key][1][0]
        g_target_twist.angular.y = key_mapping[key][1][1]
        g_target_twist.angular.z = key_mapping[key][1][2]
        rospy.loginfo("Sending twist: %s", g_target_twist)


if __name__ == '__main__':
    rospy.init_node("keystroke_to_cmd_vel")

    rospy.Subscriber("keystroke", String, keystroke_cb)
    cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        send_twist(cmd_vel_pub)
        rate.sleep()
