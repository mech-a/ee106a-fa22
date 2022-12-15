#!/usr/bin/env python
import rospy
import time
import pyomo.environ as pyo
import math
import subprocess
import numpy as np
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image

from intera_interface import gripper as robot_gripper
from intera_interface import Limb
import moveit_commander
import moveit_msgs.msg

# -0.5 -0.9 0.0 -1.4 0.0 -2.0 1.57 is a neutral position to begin throwing from only J5.
def go_to_joint_angles(theta0, theta1, theta3, theta5):
    subprocess.run(f"rosrun intera_examples go_to_joint_angles.py -q {theta0} {theta1} 0.0 {theta3} 0.0 {theta5} 1.57".split())  
    rospy.sleep(0.5)  # for safety

# converts homo image coords to a depth and angle to throw at
def get_cup_pos(x, y):
    assert 0 <= x <= 1 and 0 <= y <= 1 # cup_x and cup_y are ratios of the table dim
    # table dimensions in meters
    table_width = 18/39.37
    table_height = 57.5/39.37
    # dist from robot to table start in meters
    table_depth = 1.32 #52/39.37
    # offsets in meters
    arm_off = 0.1603  # pivot to arm
    pivot_off = 2.5/39.37 + table_width/2  # left edge of table to pivot (cuz Billy is Billy)
    
    # get real world cup position
    cup_y = table_depth + table_height * (1 - y)
    cup_x = table_width * x
    
    cup_pivot_dist = ((cup_x - pivot_off)**2 + cup_y**2) ** 0.5
    cup_depth = (cup_pivot_dist**2 - arm_off**2) ** 0.5
    
    cup_pivot_angle = math.atan(cup_x/cup_y)
    cup_pivot_rotated_angle = math.atan(arm_off/cup_depth)
    cup_angle = cup_pivot_angle - cup_pivot_rotated_angle
    return cup_depth, cup_angle

kps = []
captured = False

def capture_img(img_message):
    global kps
    global captured
    # if captured:
    #     return kps
    bridge = CvBridge()
    img_cv = bridge.imgmsg_to_cv2(img_message, desired_encoding='passthrough')
    img_ho = homograph_img(img_cv)
    kops = find_keypoints_blob(img_ho)
    if len(kops) and not captured:
        kps = kops
        cv2.imshow('img_cv', img_cv)
        cv2.imshow('img_ho', img_ho)
        img_lab = cv2.drawKeypoints(img_cv, kps, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        cv2.imshow('im_lab', img_lab)
        captured = True
        # print([kp.pt for kp in kps])

def homograph_img(img_cv):
    dist_height = 575.0 # measured: 550
    dist_width = 180.0
    pts_src = np.array([[334.0, 40.0], [578.0,44.0], [351.0,447.0],[478.0,447.0]])
    pts_dst = np.array([[0.0, 0.0], [dist_width,0.0], [0.0,dist_height],[dist_width, dist_height]])

    #---- Framing the homography matrix
    h, status = cv2.findHomography(pts_src, pts_dst)
     
    #---- transforming the image bound in the rectangle to straighten
    homo_resized = cv2.warpPerspective(img_cv, h, (int(dist_width),int(dist_height)))
    return homo_resized


def find_keypoints_blob(img_cv):
    # Set up the detector with default parameters.
    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 10
    params.maxThreshold = 300


    # Filter by Area.
    params.filterByArea = True
    params.minArea = 1500

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.05

    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.87

    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.01
    detector = cv2.SimpleBlobDetector_create(params)
    # Detect blobs.
    keypoints = detector.detect(img_cv)
    return keypoints

def main():
    rospy.init_node('throwertest')
    #rospy.Subscriber("/io/internal_camera/right_hand_camera/image_rect", Image, capture_img)

    limb = Limb('right')
    gripper = robot_gripper.Gripper('right_gripper')
    limb.set_joint_position_speed(1.0) # this only sets it for position control - i don't think it matters for us
    limb.set_command_timeout(0.4)

    # _ = input('to open')
    # #gripper.calibrate()
    # gripper.open()
    # rospy.sleep(1)

    # _ = input('to close')
    # print("Gripper is_ready:", gripper.is_ready())
    # gripper.close()
    # rospy.sleep(1)
    # #gripper.stop()
    # print("Gripper force:", gripper.get_force())
    # print("Gripper error:", gripper.has_error())
    # print("Gripper is_calibrated:", gripper.is_calibrated())
    # print("Gripper is_gripping:", gripper.is_gripping())

    # gripper.reboot()
    # gripper.set_holding_force()
    
    #_ = input('to open')
    #gripper.calibrate()
    # demjuice
    #juice = [(1.7, 0), (1.4, -0.1), (1.5, -0.05)]
    # JUICE FOR consistency
    juice = [(1.7, 0), (1.7, 0), (1.7, 0)]
    for j in juice:
        c = input('to close')
        print("Gripper is_ready:", gripper.is_ready())
        if (c != "n"):
            gripper.open()
            rospy.sleep(1)
        gripper.close()
        rospy.sleep(1)
        #gripper.stop() DO NOT USE STOP
        print("Gripper force:", gripper.get_force())
        print("Gripper error:", gripper.has_error())
        print("Gripper is_calibrated:", gripper.is_calibrated())
        print("Gripper is_gripping:", gripper.is_gripping())

        _ = input('to calculate')
        wrist_joint = 'right_j5'
        elbow_joint = 'right_j3'
        shoulder_joint = 'right_j1'

        base_height = 1.237
        hand_length = 0.3
        wrist_speed = 3.485
        forearm_length = 0.4
        elbow_speed = 1.957
        arm_length = 0.4
        shoulder_speed = 1.328

        go_to_joint_angles(-0.5, -0.785, 0, 0)

        #depth, theta = get_cup_pos(0.7, 24/57.5)
        print(kps)
        #depth, theta = kps[0].pt
        #print(depth, theta)
        depth, theta = j
        print("Throwing to depth:", depth, "meters")
        theta0_0 = -0.5 + theta  # including initial robot angle
        #depth = (56 + 0)/39.37

        height = 0.72 + 4.75/39.37 # for cup height
        g = 9.81

        model = pyo.ConcreteModel()
        model.theta1 = pyo.Var()
        model.theta3 = pyo.Var()
        model.theta5 = pyo.Var()
        model.dr = pyo.Var() # end effector depth
        model.hr = pyo.Var() # end effector height
        model.vd = pyo.Var() # end effector velocity in the x (depth) direction
        model.vh = pyo.Var() # end effector velocity in the z (height) direction
        model.t = pyo.Var() # time

        model.Constraint1 = pyo.Constraint(
            expr = model.vd == (arm_length * pyo.cos(math.pi/2 + model.theta1) * shoulder_speed) +
                               (forearm_length * pyo.cos(math.pi/2 + model.theta1 + model.theta3) * (shoulder_speed + elbow_speed)) +
                               (hand_length * pyo.cos(math.pi/2 + model.theta1 + model.theta3 +  model.theta5) * (shoulder_speed + elbow_speed + wrist_speed))
                               ) # set vd as a function of sawyer dimensions, sawyer speeds, joint angles
        model.Constraint2 = pyo.Constraint(
            expr = model.dr == (arm_length * pyo.sin(math.pi/2 + model.theta1)) +
                               (forearm_length * pyo.sin(math.pi/2 + model.theta1 + model.theta3)) +
                               (hand_length * pyo.sin(math.pi/2 + model.theta1 + model.theta3 +  model.theta5)) 
                               ) # set dr as a function of sawyer dimensions, joint angles
        model.Constraint3 = pyo.Constraint(
            expr = model.hr == (arm_length * pyo.cos(math.pi/2 + model.theta1)) +
                               (forearm_length * pyo.cos(math.pi/2 + model.theta1 + model.theta3)) +
                               (hand_length * pyo.cos(math.pi/2 + model.theta1 + model.theta3 +  model.theta5)) 
                               ) # set hr as a function of sawyer dimensions, joint angles
        model.Constraint4 = pyo.Constraint(
            expr = model.vh == -((arm_length * pyo.sin(math.pi/2 + model.theta1) * shoulder_speed) +
                               (forearm_length * pyo.sin(math.pi/2 + model.theta1 + model.theta3) * (shoulder_speed + elbow_speed)) +
                               (hand_length * pyo.sin(math.pi/2 + model.theta1 + model.theta3 +  model.theta5) * (shoulder_speed + elbow_speed + wrist_speed))
                               )) # set vh as a function of sawyer dimensions, sawyer speeds, joint angles
        # minimize distance between target coordinates and the coordinates of the ball at some point along its trajectory
        model.Objective = pyo.Objective(expr = (model.dr + model.t*model.vd - depth)**2 + (base_height + model.hr + model.t*model.vh - g*(model.t**2)/2 - height)**2)

        solver = pyo.SolverFactory('ipopt')
        results = solver.solve(model)

        theta1 = pyo.value(model.theta1)
        theta3 = pyo.value(model.theta3)
        theta5 = pyo.value(model.theta5)
        dr = pyo.value(model.dr)
        hr = pyo.value(model.hr)
        vd = pyo.value(model.vd)

        print(f"{theta1=}, {theta3=}, {theta5=}")
        print(f"{dr=}, {hr=}, {vd=}")
        print(f"{results.solver.termination_condition=}")
        # model.display()

        _ = input('to load')
        total_time = 0.4
        throw_time = 0.3
        theta1_0 = theta1 - throw_time * shoulder_speed
        theta3_0 = theta3 - throw_time * elbow_speed
        theta5_0 = theta5 - throw_time * wrist_speed
        print(f"{theta1_0=}, {theta3_0=}, {theta5_0=}")
        
        go_to_joint_angles(theta0_0, theta1_0, theta3_0, theta5_0)
        
        _ = input('to throw')
        t0 = time.time()
        while not rospy.is_shutdown():
            limb.set_joint_velocities({elbow_joint: elbow_speed, wrist_joint: wrist_speed, shoulder_joint: shoulder_speed})
            d = time.time() - t0
            if d >= throw_time:
                break
        print(f"{time.time()-t0=}, {d=}")
        gripper.open()
        print(f"{time.time()-t0=}, {d=}")
        while not rospy.is_shutdown():
            limb.set_joint_velocities({elbow_joint: elbow_speed, wrist_joint: wrist_speed, shoulder_joint: shoulder_speed})
            d = time.time() - t0
            if d >= total_time:
                break
        print('done!')


if __name__ == '__main__':
    main()
