#!/usr/bin/env python
import rospy
import time
import pyomo.environ as pyo
import math

from intera_interface import gripper as robot_gripper
from intera_interface import Limb
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys
from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject


# -0.5 -0.9 0.0 -1.4 0.0 -2.0 1.57 is a neutral position to begin throwing from only J5.


# moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("moveit_move_group_test")
# robot = moveit_commander.RobotCommander()
# move_group = moveit_commander.MoveGroupCommander('right_arm')

display_trajectory_publisher = rospy.Publisher(
	"/move_group/display_planned_path",
	moveit_msgs.msg.DisplayTrajectory,
	queue_size=20
)

# move_group.set_max_velocity_scaling_factor(0.1)
# joint_goal = move_group.get_current_joint_values()
# joint_goal[1] = 0
# move_group.go(joint_goal, wait=True)
# move_group.stop()

group_name = 'right_arm'
# Initialize moveit_commander
moveit_commander.roscpp_initialize(sys.argv)

# Initialize the robot
robot = moveit_commander.RobotCommander()

# Initialize the planning scene
scene = moveit_commander.PlanningSceneInterface()

# This publishes updates to the planning scene
planning_scene_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)

# Instantiate a move group
group = moveit_commander.MoveGroupCommander(group_name)

# Set the maximum time MoveIt will try to plan before giving up
group.set_planning_time(5)

# Set the bounds of the workspace
group.set_workspace([-2, -2, -2, 2, 2, 2])

# Sleep for a bit to ensure that all inititialization has finished
# rospy.sleep(0.5)

group.set_start_state_to_current_state()
group.set_joint_value_target('right_j3', 1)
plan = group.plan()

_ = input('waiting')

group.execute(plan[1])