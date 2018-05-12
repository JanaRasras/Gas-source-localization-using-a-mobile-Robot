#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
f = open('L.txt','w')
def scan_callback(msg):
 
    range_ahead = msg.ranges[len(msg.ranges)/-4]
    range_ahead2 = msg.ranges[len(msg.ranges)/18]
    range_ahead3 = msg.ranges[len(msg.ranges)/8]
    range_ahead4 = msg.ranges[len(msg.ranges)/6]
    range_ahead5 = msg.ranges[len(msg.ranges)/4]
    range_ahead6 = msg.ranges[len(msg.ranges)/2]
    print range_ahead,range_ahead2,range_ahead3,range_ahead4,range_ahead5,range_ahead6
    f.write( " %0.12f"  % range_ahead )
    f.write('\t')
    f.write( " %0.12f"  % range_ahead2 )
    f.write('\t')
    f.write( " %0.12f"  % range_ahead3 )
    f.write('\t')
    f.write( " %0.12f"  % range_ahead4)
    f.write('\t')
    f.write( " %0.12f"  % range_ahead5 )
    f.write('\t')
    f.write( " %0.12f"  % range_ahead6 )
    f.write('\t')
    f.write('\n')

rospy.init_node('range_ahead')
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
rospy.spin()


