#!/usr/bin/env python

import rospy
import math
import numpy
import numpy as np
from nav_msgs.msg import Odometry
import serial
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
import time
import random

class MoveBaseSeq():

    global th,a,f,f2,x,y,g,m,xm,ym,data
    th = 0.1
    a = a = serial.Serial('/dev/ttyACM1', 9600)
    f  = open('data6.txt','w')
    f2 = open('seq6.txt','w')
    x = []
    y = []
    g = []
    m  = 0
    xm = 0
    ym = 0
    data = 0

    def __init__(self):

        rospy.init_node('move_base_sequence')
        rospy.Subscriber('/odom', Odometry,self.callback)

        points_seq = ([])
        points_seq.append(0)
        points_seq.append(0)
        points_seq.append(0)
        for i in range(1,45):
            x1 = random.uniform(0,5)+0.01
            y1 = random.uniform(0.0,3.0)+0.01
            points_seq.append(x1)
            points_seq.append(y1)
            points_seq.append(0)
        points_seq.append(4.3)
        points_seq.append(2)
        points_seq.append(0)
        points_seq.append(4.3)
        points_seq.append(1.5)
        points_seq.append(0)
        points_seq.append(0)
        points_seq.append(4.2)
        points_seq.append(2)
        points_seq.append(0)
        points_seq.append(0)
        points_seq.append(3.5)
        points_seq.append(2)
        points_seq.append(0)
        points_seq.append(0)
        f2.write(str(points_seq))  
        f2.close()  
        
        yaweulerangles_seq =[0,270,0,270,0,270,0,270,0,270,0,0,270,0,270,0,270,0,270,0,270,0,0,270,0,270,0,270,0,270,0,270,0,0,270,0,270,0,270,0,270,0,270,0,0,270,0,270,0,270,0,270,0,270,0,0,270,0,270,0,270,0,270,0,270,0,0,270,0,270,0,270,0,270,0,270,0,0,270,0,270,0,270,0,270,0,270,0,0,270,0,270,0,270,0,270,0,270,0,0,270,0,270,0,270,0,270,0,270,0,0,270,0,270,0,270,0,270,0,270,0,0,270,0,270,0,270,0,270,0,270,0,0,270,0,270,0,270,0,270,0,270,0,0,270,0,270,0,270] 
       
        quat_seq = list()

        self.pose_seq = list()
        self.goal_cnt = 0
        self.cnt = 0
        for yawangle in yaweulerangles_seq:

            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        n = 3

        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        rospy.loginfo(str(points))
        for point in points:

            self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
            n += 1

        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.movebase_client()

    def callback(self,msg):
        global fx,fy,fg, data
        data = (a.readline().strip())
        fx = msg.pose.pose.position.x
        fy = msg.pose.pose.position.y
        fg = data


    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):

        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")

    def done_cb(self, status, result):
        global m,xm,ym

        self.goal_cnt += 1

        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached") 
            if self.goal_cnt< len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()

                time.sleep(30)
                if(float(data) >th):
                    x.append(fx)
                    y.append(fy)
                    g.append(fg)
	            f.write(str(self.cnt))
	            f.write('\t')
                    f.write(str(x[self.cnt-1]))
		    f.write('\t')
		    f.write(str(y[self.cnt-1]))
		    f.write('\t')
		    f.write(str(g[self.cnt-1]))
		    f.write('\n')
                    print g[self.cnt-1] 
                for i in range(len(g)):
                    if(g[i]>m):
                        m  = g[i]
                        xm = x[i]
                        ym = y[i]
                        
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb) 

            else:
                points_seq = [xm,ym,0]
                yaweulerangles_seq = [0]
                quat_seq = list()
                self.pose_seq = list()
                self.goal_cnt = 0
                for yawangle in yaweulerangles_seq:
                    quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
                n = 3
                points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
                for point in points:
                    self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
                    n += 1
                    self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
                    self.movebase_client()
                    if self.goal_cnt< len(self.pose_seq):
                        next_goal = MoveBaseGoal()
                        next_goal.target_pose.header.frame_id = "map"
                        next_goal.target_pose.header.stamp = rospy.Time.now()
                        next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                        self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)
                f.close()  
                return

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            if self.goal_cnt< len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)
            else:
                points_seq = [xm,ym,0]
                yaweulerangles_seq = [0]
                quat_seq = list()
                self.pose_seq = list()
                self.goal_cnt = 0
                for yawangle in yaweulerangles_seq:
                    quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
                n = 3
                points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
                for point in points:
                    self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
                    n += 1
                    self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
                    self.movebase_client()
                    if self.goal_cnt< len(self.pose_seq):
                        next_goal = MoveBaseGoal()
                        next_goal.target_pose.header.frame_id = "map"
                        next_goal.target_pose.header.stamp = rospy.Time.now()
                        next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                        self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)
                f.close()  
                return

            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()

if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
