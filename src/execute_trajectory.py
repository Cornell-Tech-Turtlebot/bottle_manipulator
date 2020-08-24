#! /usr/bin/env python

#Code taken from Robot Ignite Academy. If perfroms multiple movements in the joint task space. The arm
# will move from the home position to a position above a 40x40mm cube, the pick the cube up, return home, 
# then put the cube bacl where it got it from.

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

names1 = 'position1'
values1 = [0.1,0.85,0.1,-.90]
names2 = 'position2'
values2 = [0.6,-0.85,0.9,0]

names3 = 'position3'
values3 = [0.0,0.0,-0.92,0.62]

###### Functions ########
def open_gripper():
	print "Opening Gripper..."
	gripper_group_variable_values[0] = 00.017
        print "Type Gripper Group Values", type(gripper_group_variable_values)
	gripper_group.set_joint_value_target(gripper_group_variable_values)
	plan2 = gripper_group.go()
	gripper_group.stop()
	gripper_group.clear_pose_targets()
	rospy.sleep(1)

def close_gripper():
	rospy.sleep(2)
	print "Closing Gripper..."
	gripper_group_variable_values[0] = .005
	gripper_group.set_joint_value_target(gripper_group_variable_values)
	plan2 = gripper_group.go()
	gripper_group.stop()
	gripper_group.clear_pose_targets()
	rospy.sleep(1)

def move_home():
	arm_group.set_named_target("home")
	print "Executing Move: Home"
	plan1 = arm_group.plan()
	if not arm_group.execute(plan1, wait=True):
            move_home()
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print (variable.pose)
	rospy.sleep(2)
	
def move_zero():
	arm_group.set_named_target("zero")
	print "Executing Move: Zero"
	plan1 = arm_group.plan()
	arm_group.execute(plan1, wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print (variable.pose)
	rospy.sleep(1)

def move_position1(): 
            
	arm_group.set_named_target("position1")
	print "Executing Move: Position1"
	plan1 = arm_group.plan()
	if (not arm_group.execute(plan1, wait=True)) and (arm_group.get_current_pose().pose.position.z > 0.18):
            move_position1()
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print (variable.pose)
	rospy.sleep(2)

def move_position2():
	arm_group.set_named_target("position2")
	print "Executing Move: Position2"
	plan1 = arm_group.plan()
	if not arm_group.execute(plan1, wait=True):
            move_position2()
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print (variable.pose)
	rospy.sleep(3)

def move_position3():
	arm_group.set_named_target("position3")
	print "Executing Move: Position3"
	plan1 = arm_group.plan()
	if not arm_group.execute(plan1, wait=True):
            move_position3()
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print (variable.pose)
        

	rospy.sleep(2)

###### Setup ########
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_execute_trajectory', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm_group = moveit_commander.MoveGroupCommander("arm")

#Had probelms with planner failing, Using this planner now. I believe default is OMPL
arm_group.set_planner_id("RRTConnectkConfigDefault")
#Increased available planning time from 5 to 10 seconds
arm_group.set_planning_time(50);

gripper_group = moveit_commander.MoveGroupCommander("gripper")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)



arm_group.remember_joint_values(names1, values1)
arm_group.remember_joint_values(names2, values2)
arm_group.remember_joint_values(names3, values3)

gripper_group_variable_values = gripper_group.get_current_joint_values()

###### Main ########
picked = False
move_home()
open_gripper()
start = 1000
move_position1()
close_gripper()
picked = True
#move_home()


move_position2()

move_position3()

#open_gripper()
move_home()

move_position1()

open_gripper()

move_home()


moveit_commander.roscpp_shutdown()
