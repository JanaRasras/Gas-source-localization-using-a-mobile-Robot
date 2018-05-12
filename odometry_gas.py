#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import serial

a = serial.Serial('/dev/ttyACM1', 9600)


def callback(msg):
    data = (a.readline().strip())
    print msg.pose.pose.position.x , msg.pose.pose.position.y, msg.pose.pose.orientation.w , data
    
rospy.init_node('check_odometry')
odom_sub = rospy.Subscriber('/odom', Odometry, callback)
rospy.spin()
