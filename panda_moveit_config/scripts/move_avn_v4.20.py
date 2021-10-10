#!/usr/bin/env python

import os
import tf
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
        
        moveit_commander.roscpp_initialize(sys.argv)                               
        rospy.init_node('move_avn',anonymous=True)                                 

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()
        
        group_name = "panda_arm"
        group = moveit_commander.MoveGroupCommander(group_name)
        
        group.set_pose_reference_frame("panda_link_0")

        ## We create a `DisplayTrajectory`_ publisher which is used later to publish trajectories for RViz to visualize:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
        
        unused_variable = os.system("clear")

        planning_frame = group.get_planning_frame()
        print "-----Planning Frame:----- %s" % planning_frame
        eef_link = group.get_end_effector_link()           
        print "-----End effector:----- %s" % eef_link 
        current_state = robot.get_current_state()
        print "------currant state:----- %s" %current_state
        currant_rpy = group.get_current_rpy(end_effector_link = "tool0")
        print "-----currant rpy----- %s" %currant_rpy
        currant_pose = group.get_current_pose(end_effector_link = "tool0")
        print "-----currant pose----- %s" %currant_pose

        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link

    def print_general_info(self):
        os.system("clear")   
        group = self.group
        planning_frame = group.get_planning_frame()
        print "Planning Frame: %s" % planning_frame
        eef_link = group.get_end_effector_link()  
        print "End effector: %s" % eef_link 
        current_state = robot.get_current_state()
        print "currant state: %s" %current_state


    def go_to_sleep_position(self):                                           #function for taking robot to home position
        group = self.group
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = math.radians(-95.59)
        joint_goal[1] = math.radians(-1.26)
        joint_goal[2] = math.radians(12.15)
        joint_goal[3] = math.radians(0.00)
        joint_goal[4] = math.radians(100.00)
        joint_goal[5] = math.radians(0.00)
    
        group.go(joint_goal, wait=True)

        group.stop()
        rospy.sleep(1)	
        current_joints = self.group.get_current_joint_values()
        print [math.degrees(joint_val) for joint_val in current_joints]
        return

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
    
        display_trajectory_publisher.publish(display_trajectory) 

    def draw_visualisation_marker(self,wpose):
        global draw_marker
        marker = Marker()
        marker.header.frame_id = "/base"
        marker.type = marker.ARROW
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.pose.orientation.x = wpose.pose.orientation.x
        marker.pose.orientation.y = wpose.pose.orientation.y
        marker.pose.orientation.z = wpose.pose.orientation.z
        marker.pose.orientation.w = wpose.pose.orientation.w
        marker.pose.position.x = wpose.pose.position.x
        marker.pose.position.y = wpose.pose.position.y
        marker.pose.position.z = wpose.pose.position.z
        draw_marker.publish(marker)

    def execute(self,target):
        global object_frame_pub
        group = self.group
        robot = self.robot
        current_state = robot.get_current_state()
        wpose = geometry_msgs.msg.PoseStamped()
        wpose.header.stamp = rospy.Time()
        wpose.header.frame_id = "/base"
        euler = tf.transformations.euler_from_quaternion((target.orientation.x,target.orientation.y,target.orientation.z,target.orientation.w), axes = "sxyz")

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
            print"The trajectory execution aborted due to incorrect target points.....No plan made"
            return 0
        else: 	
            self.display_trajectory(plan)
            print"Plan made...Starting Execution"
            a = raw_input()
            if a == 'y':
                group.execute(plan,wait=True)
                print"Executed the Target"
                rospy.sleep(0.1)
            return 1 
    

def move_abb_callback(target_array):
    global abb
    global start_scanning
    #abb.go_to_sleep_position()
    abb.go_to_wake_position()
    no_picked=0
    print(target_array)
    for i in range(len(target_array.poses)):
        abb.go_to_wake_position()
        no_picked += abb.execute(target_array.poses[i])
        print i
        #raw_input()
        #if picked == 1:
        #    break
        #else:
        #    abb.go_to_wake_position()
        #    continue
        abb.go_to_wake_position()
        abb.go_to_sleep_position()
        rospy.sleep(0.1)
        abb.turn_vacuum_off()
    #abb.go_to_sleep_position()
    abb.turn_vacuum_off()
    start_scanning.publish(1.0) 

if __name__ == '__main__':

    picked = 0    
    abb = moveit_planning()
    robot_state_pub = rospy.Publisher('robot_state',std_msgs.msg.Int32,queue_size=10)
    start_scanning = rospy.Publisher('/plc_modbus_control/scan_forward',std_msgs.msg.Float64,queue_size=1)
    #object_frame_pub = rospy.Publisher('/object_frame_publisher', PoseStamped, queue_size = 1)
    draw_marker = rospy.Publisher('marker',Marker,queue_size = 1)
    rospy.loginfo("________PRESS ENTER TO START THE MOTION____________")
    raw_input()
    rospy.Subscriber("target_pose_array",PoseArray,move_abb_callback)
    rospy.Subscriber("io_pins_status]", Int32, picked_status)

    while not rospy.is_shutdown():
        start_scanning.publish(1.0)
        rospy.spin()
        
        #io_status = rospy.wait_for_message("/io_pins_status",std_msgs.msg.Int32)
        #if io_status == 1 : robot_state_pub.publish(picked)
        #else : robot_state_pub.publish(not_picked)
        #abb.turn_vacuum_off()
             #cartesian_plan, fraction = abb.plan_cartesian_path()
       #
    #abb.execute_plan(cartesian_plan)
    #set_rpy_target(self,rpy,end_effector_link = "") 		
