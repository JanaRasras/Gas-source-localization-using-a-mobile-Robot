#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import serial

a = serial.Serial('/dev/ttyACM1', 9600)
f= open('a5.txt','w')

def callback(msg):
    data = (a.readline().strip())
    x = msg.pose.pose.position.x 
    print msg.pose.pose.position.x , msg.pose.pose.position.y, data
   # f.write(str(msg.pose.pose.position.x ),str( msg.pose.pose.position.y), data)
    f.write("%0.12f" % msg.pose.pose.position.x) 
    f.write('\t')
    f.write("%0.12f" % msg.pose.pose.position.y)
    f.write('\t')
    f.write( data)
    f.write('\n')
rospy.init_node('check_odometry')
odom_sub = rospy.Subscriber('/odom', Odometry, callback)
rospy.spin()
f.close()
