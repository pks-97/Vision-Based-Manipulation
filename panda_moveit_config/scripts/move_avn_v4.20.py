#!/usr/bin/env python

import os
import sys
import math
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
from geometry_msgs.msg import PoseStamped,PoseArray
from math import pi
from std_msgs.msg import String, Float64, Int32
from moveit_commander.conversions import pose_to_list
# visualisation marker
from visualization_msgs.msg import Marker

class moveit_planning(object):

    def __init__(self):                                                             
        super(moveit_planning, self).__init__()
        
        moveit_commander.roscpp_initialize(['joint_states:=/panda/joint_states'])
        rospy.init_node('move_avn',anonymous=True)                                 

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "panda_arm"
        group = moveit_commander.MoveGroupCommander(group_name)
        #group.set_pose_reference_frame("world")
        
        current_state = robot.get_current_state()
        print("------currant state:----- %s" %current_state)
        planning_frame = group.get_planning_frame()
        print("-----Planning Frame:----- %s" % planning_frame)
        eef_link = group.get_end_effector_link()           
        print("-----End effector:----- %s" % eef_link)
        currant_rpy = group.get_current_rpy(end_effector_link = "panda_link7")
        print("-----currant rpy----- %s" %currant_rpy)
        currant_pose = group.get_current_pose(end_effector_link = "panda_link7")
        print("-----currant pose----- %s" %currant_pose)

        self.robot = robot
        self.scene = scene
        self.group = group
        self.planning_frame = planning_frame
        self.eef_link = eef_link

    def print_general_info(self):
        group = self.group
        planning_frame = group.get_planning_frame()
        print("Planning Frame: %s" % planning_frame)
        eef_link = group.get_end_effector_link()  
        print("End effector: %s" % eef_link)
        current_state = robot.get_current_state()
        print("currant state: %s" %current_state)


    def go_to_sleep_position(self):                                           #function for taking robot to home position
        group = self.group
        joint_goal = group.get_current_joint_values()
        print(joint_goal)
        joint_goal[0] = -0.63
        joint_goal[1] = 0.69
        joint_goal[2] = 1.35
        joint_goal[3] = -0.96
        joint_goal[4] = -0.88
        joint_goal[5] = 1.21
        joint_goal[6] = 0.41
    
        group.go(joint_goal, wait=True)

        group.stop()
        rospy.sleep(1)
        return

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
    
        display_trajectory_publisher.publish(display_trajectory) 

    def execute(self):
        global object_frame_pub
        group = self.group
        robot = self.robot
        current_state = robot.get_current_state()
        wpose = geometry_msgs.msg.PoseStamped()
        wpose.header.stamp = rospy.Time()
        wpose.header.frame_id = "/world"

        print("original_target_XYZ",target.position)
        wpose.pose.position.x = target.position.x + 0.065
        wpose.pose.position.y = target.position.y  - 0.025
        wpose.pose.position.z = target.position.z + 0.02
        wpose.pose.orientation.x = 0 #target.orientation.x
        wpose.pose.orientation.y = 1 #target.orientation.y
        wpose.pose.orientation.z = 0 #target.orientation.z
        wpose.pose.orientation.w = 0 #target.orientation.w
        #object_frame_pub,publish(wpose)
        print("Planning for the target :--> ")
        print(wpose)
        self.draw_visualisation_marker(wpose)
        group.set_planner_id("RRTConnectkConfigDefault")
        group.set_planning_time(5)
        group.set_start_state(current_state)
        group.set_pose_target(wpose,"tool0")
        plan = group.plan()
        if len(plan.joint_trajectory.points)==0: 	
            return 0
        else: 	
            print("Plan made...Starting Execution")
            a = raw_input()
            if a == 'y':
                group.execute(plan,wait=True)
                print("Executed the Target")
                rospy.sleep(0.1)
            return 1 
    

def move_abb_callback():
    global abb
    abb.go_to_sleep_position()
    #abb.execute()

if __name__ == '__main__':
    abb = moveit_planning()
    move_abb_callback()
    rospy.loginfo("________PRESS ENTER TO START THE MOTION____________")
    while not rospy.is_shutdown():
        rospy.spin()
