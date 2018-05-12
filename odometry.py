#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import serial


def callback(msg):

    print(msg.pose.pose.position.x , msg.pose.pose.position.y,msg.pose.pose.position.z)

rospy.init_node('check_odometry')
odom_sub = rospy.Subscriber('/odom', Odometry, callback)
rospy.spin()
