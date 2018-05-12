#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import serial

a = serial.Serial('/dev/ttyACM0', 9600)

x = []
y = []
g = []
def callback(msg): 
    data = (a.readline().strip())
    fx = [msg.pose.pose.position.x]
    fy = [msg.pose.pose.position.y] 
    fg = [data]
    x.append(fx)
    y.append(fy)
    g.append(fg)   
    max = x[0] 
    xf = x[0]
    yf = y[0]
    for i in range(len(g)):
        if(g[i]>max):
            max=g[i]
            xf=x[i]
            yf=y[i]
    print (msg.pose.pose.position.x,msg.pose.pose.position.y,data)
    print  max  
    print xf 
    print yf
rospy.init_node('check_odometry')
odom_sub = rospy.Subscriber('/odom', Odometry, callback)
rospy.spin()
