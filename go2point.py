#!/usr/bin/env python

#--------Include modules-----------------------------------------------
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from time import sleep

rospy.init_node('test', anonymous=False) 
pub =  rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10) 
sleep(1)

target = PoseStamped()
target.header.frame_id = "map"
target.header.stamp = rospy.Time.now()

target.pose.orientation.x = 0.0
target.pose.orientation.y = 0.0
target.pose.orientation.z = 0.0
target.pose.orientation.w = 1.0


target.pose.position.x = 12.0
target.pose.position.y = 12.0
pub.publish(target)

