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
import sensor_msgs.point_cloud2 as pc2
# visualisation marker
from visualization_msgs.msg import Marker


rospy.init_node('move_avn',anonymous=True)
pub = rospy.Publisher('/panda/panda_finger1_controller/command', Float64, queue_size=10)
pub1 = rospy.Publisher('/panda/panda_finger2_controller/command', Float64, queue_size=10)
class moveit_planning(object):

    def __init__(self):                                                             
        super(moveit_planning, self).__init__()
        
        moveit_commander.roscpp_initialize(['joint_states:=/panda/joint_states'])
        # rospy.init_node('move_avn',anonymous=True)                                 

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
        joint_goal[0] = 2.328	
        joint_goal[1] = -0.85
        joint_goal[2] = -1.39
        joint_goal[3] = -1.042
        joint_goal[4] = -0.94
        joint_goal[5] = 1.191
        joint_goal[6] = 0.98
    
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

  # position: 
  #   x: 0.6417985558509827
  #   y: 0.014854976907372475
  #   z: 0.6817941665649414
  # orientation: 
  #   x: 0.34433943033218384
  #   y: 0.9358659386634827
  #   z: -0.026344887912273407
  #   w: 0.02726620249450206

  # position: 
  #   x: 0.6425893902778625
  #   y: 0.013632533140480518
  #   z: 0.6831778287887573
  # orientation: 
  #   x: 0.25365152955055237
  #   y: 0.9664899706840515
  #   z: 0.014226187951862812
  #   w: -0.013217533007264137


        # print("original_target_XYZ",target.position)
        wpose.pose.position.x= 0.6425893902778625
        wpose.pose.position.y= 0.013632533140480518
        wpose.pose.position.z = 0.6831778287887573
        wpose.pose.orientation.x = 0.25365152955055237 #target.orientation.x
        wpose.pose.orientation.y = 0.9664899706840515 #target.orientation.y
        wpose.pose.orientation.z = 0.014226187951862812 #target.orientation.z
        wpose.pose.orientation.w = -0.013217533007264137 #target.orientation.w

        #object_frame_pub,publish(wpose)
        print("Planning for the target :--> ")
        print(wpose)
        group.set_planner_id("RRTConnectkConfigDefault")
        group.set_planning_time(10)
        group.set_start_state(current_state)
        group.set_pose_target(wpose,"panda_link7")
        group.set_goal_tolerance(0.001)
        plan = group.plan()
        print("Pratyush")
        print(plan[1].joint_trajectory)
        print("Plan made...Starting Execution")
        # group.execute(plan[1].joint_trajectory.points,wait=True)
        group.go(wait=True)
        ans = Float64()
        ans.data = 0.1
        pub.publish(ans)
        pub1.publish(ans)
        print("Executed the Target")
        rospy.sleep(0.1)
        return 1 
    
    def moveRight(self):
        scale = 1.0
        global object_frame_pub
        group = self.group
        robot = self.robot
        waypoints = []
        current_state = robot.get_current_state()
        wpose = group.get_current_pose().pose
        wpose.position.x += scale * 0.5
        wpose.position.y += scale * 0.0
        wpose.position.z += scale * 0.0
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = group.compute_cartesian_path(
    waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
)
        group.execute(plan, wait=True)
        # js = rospy.wait_for_message('/panda_camera/depth/points', pc2)
        return 1 

    def moveLeft(self):
        scale = 1.0
        global object_frame_pub
        group = self.group
        robot = self.robot
        waypoints = []
        current_state = robot.get_current_state()
        wpose = group.get_current_pose().pose
        wpose.position.y += scale * (-0.2)
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = group.compute_cartesian_path(
    waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
)
        group.execute(plan, wait=True)
        # js = rospy.wait_for_message('/panda_camera/depth/points', pc2)
        return 1 

def move_abb_callback():
    global abb
    # abb.go_to_sleep_position()
    abb.execute()
    # abb.moveRight()
    # abb.moveLeft()

if __name__ == '__main__':
    abb = moveit_planning()
    move_abb_callback()
    rospy.loginfo("________PRESS ENTER TO START THE MOTION____________")
    while not rospy.is_shutdown():
        rospy.spin()

