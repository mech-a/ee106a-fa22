#!/usr/bin/env python
import rospy
import time

from intera_interface import gripper as robot_gripper
from intera_interface import Limb

# -0.5 0.0 0.0 0.0 0.0 -2.96 1.57 is a neutral position to begin throwing from only J5.

def main():
    rospy.init_node('throwertest')
    limb = Limb('right')
    gripper = robot_gripper.Gripper('right_gripper')
    limb.set_joint_position_speed(1.0) # this only sets it for position control - i don't think it matters for us
    # alternatively, we could try position control
    limb.set_command_timeout(0.4)

    # _ = input('go to defaults')

    # default_angles = dict(zip(limb.joint_names(), [-0.5, 0.0, 0.0, -1.6, 0.0, -2.96, 1.57]))

    # limb.set_joint_positions(default_angles)


    # print(limb.joint_names())
    _ = input('to calibrate')

    gripper.calibrate()
    # rospy.sleep(2.0)
    _ = input('to close')
    gripper.close()
    # rospy.sleep(2.0)

    _ = input('to throw')
    wrist_joint = 'right_j5'
    elbow_joint = 'right_j3'
    shoulder_joint = 'right_j1'


    # t0 = time.time()
    # total_time = 0.25
    # thresh = .1
    # while not rospy.is_shutdown():
    #     # limb.set_joint_velocities({elbow_joint: 6.0, wrist_joint: 4.0})
    #     limb.set_joint_positions({elbow_joint: -1.0, wrist_joint: -1.0})
    #     # if time.time() - t0 >= total_time:
    #     if abs(limb.joint_angle(elbow_joint) + 1) < thresh and abs(limb.joint_angle(wrist_joint) + 1) < thresh:
    #         break
    
    # t0 = time.time()
    # total_time = 0.5
    # throw_time = 0.37
    # while not rospy.is_shutdown():
    #     limb.set_joint_velocities({elbow_joint: 6.0, wrist_joint: 4.0})
    #     # limb.set_joint_positions({elbow_joint: 0.0, wrist_joint: 0.0})
    #     d = time.time() - t0
    #     if d >= total_time:
    #         break
    #     elif d >= throw_time:
    #         gripper.open()
    # # rospy.sleep(1.0)
    # limb.set_joint_position_speed(0.5)
    # print('done!')

    
    t0 = time.time()
    # # CONSISTENT THROWDOWN
    # total_time = 0.5
    # throw_time = 0.37
    ## LESS CONSISTENT THROWDOWN?
    total_time = 0.5
    throw_time = 0.4
    while not rospy.is_shutdown():
        # limb.set_joint_velocities({elbow_joint: 6.0, wrist_joint: 4.0, shoulder_joint: 1.0})
        limb.set_joint_velocities({elbow_joint: 6.0, wrist_joint: 6.0, shoulder_joint: 6.0})
        # limb.set_joint_velocities({shoulder_joint: 1.0})
        # limb.set_joint_positions({elbow_joint: 0.0, wrist_joint: 0.0})
        d = time.time() - t0
        if d >= throw_time:
            break
    gripper.open()
    while not rospy.is_shutdown():
        # limb.set_joint_velocities({elbow_joint: 6.0, wrist_joint: 4.0, shoulder_joint: 1.0})
        limb.set_joint_velocities({elbow_joint: 6.0, wrist_joint: 6.0, shoulder_joint: 6.0})
        # limb.set_joint_positions({elbow_joint: 0.0, wrist_joint: 0.0})
        d = time.time() - t0
        if d >= total_time:
            break
    # rospy.sleep(1.0)
    # limb.set_joint_position_speed(0.5)
    print('done!')



if __name__ == '__main__':
    main()



# import rospy

# from intera_interface import gripper as robot_gripper

# rospy.init_node('gripper_test')

# # Set up the right gripper
# right_gripper = robot_gripper.Gripper('right_gripper')

# # Calibrate the gripper (other commands won't work unless you do this first)
# print('Calibrating...')
# right_gripper.calibrate()
# rospy.sleep(2.0)

# _ = input('to close')

# # Close the right gripper
# print('Closing...')
# right_gripper.close()
# rospy.sleep(1.0)

# _ = input('to open')

# # Open the right gripper
# print('Opening...')
# right_gripper.open()
# rospy.sleep(1.0)
# print('Done!')