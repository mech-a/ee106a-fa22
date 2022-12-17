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

# -------------------------------------------------------------
# CONSTANTS
NUM_THROWS = 1
THROW_TIME = 0.4  # time in seconds of entire throwing motion
RELEASE_TIME = 0.3  # time in seconds until ball release

CUP_HEIGHT = 45  # in deci-inches
# table dimensions in deci-inches
TABLE_LENGTH = 600.0
TABLE_WIDTH = 180.0
# table position
TABLE_DEPTH = 517.5  # dist from robot pivot to table start (South edge) in deci-inches
TABLE_HEIGHT = 0.72  # height of table surface from ground
TABLE_IMG_CORNERS = np.array([[331.0, 17.0], [453.0, 17.0], [278.0,389.0],[511.0,390.0]])  # coords of table corners in image (NW, NE, SW, SE)
ARM_OFF = 0.1603  # dist between pivot and throwing axis in meters
PIVOT_OFF = 12.5/39.37  # dist between pivot and left edge of table in meters
# depth offset in perceived cup pos meters to account for minor errors (eg: overshooting -> -ve offset)
DEPTH_OFFSET = -4.5/39.37

# GLOBAL VARIABLES
kps = []  # global list for detected cup locations
captured = False  # global flag for whether image is captured 
# -------------------------------------------------------------


# run shell command to go to specified joint angle
def go_to_joint_angles(theta0, theta1, theta3, theta5):
    subprocess.run(f"rosrun intera_examples go_to_joint_angles.py -q {theta0} {theta1} 0.0 {theta3} 0.0 {theta5} 1.57".split())  
    rospy.sleep(0.5)  # for safety


# converts homo image coords (origin at NW corner) to a depth and angle to throw at
def get_cup_pos(x, y):
    # convert deci-inches to ratios
    x /= TABLE_WIDTH
    y += CUP_HEIGHT 
    y /= TABLE_LENGTH

    assert 0 <= x <= 1 and 0 <= y <= 1 # cup_x and cup_y are ratios of the table dim
    # table dimensions in meters
    table_width = TABLE_WIDTH/393.7
    table_height = TABLE_LENGTH/393.7
    # dist from robot to table start in meters
    table_depth = TABLE_DEPTH/393.7
    # get real world cup position
    cup_y = table_depth + table_height * (1 - y)
    cup_x = table_width * x
    
    # calculate cup depth
    cup_pivot_dist = ((cup_x - PIVOT_OFF)**2 + cup_y**2) ** 0.5
    cup_depth = (cup_pivot_dist**2 - ARM_OFF**2) ** 0.5
    
    # calculate cup angle (trig stuff)
    cup_pivot_angle = math.atan(cup_x/cup_y)
    cup_pivot_rotated_angle = math.atan(ARM_OFF/cup_depth)
    cup_angle = cup_pivot_angle - cup_pivot_rotated_angle

    return cup_depth + DEPTH_OFFSET, cup_angle


# callback function for subscriber - takes image message and updates global variables
def capture_img(img_message):
    global kps
    global captured
    # if image already captured, return cup locations
    if captured:
        return kps
    
    bridge = CvBridge()
    img_cv = bridge.imgmsg_to_cv2(img_message)  # convert image type
    img_ho = homograph_img(img_cv)  # perform homography on image
    kops = find_circles(img_ho)  # run circle detection to find cup locations
    if len(kops) and not captured:
        kps = kops
        captured = True


# perform homography on given CV image
def homograph_img(img_cv):
    pts_src = TABLE_IMG_CORNERS
    pts_dst = np.array([[0.0, 0.0], [TABLE_WIDTH,0.0], [0.0,TABLE_LENGTH],[TABLE_WIDTH, TABLE_LENGTH]])

    # framing the homography matrix
    h, status = cv2.findHomography(pts_src, pts_dst)
     
    # transforming the image bound in the rectangle to straighten
    homo_resized = cv2.warpPerspective(img_cv, h, (int(TABLE_WIDTH),int(TABLE_LENGTH)))
    return homo_resized


# find circle coords (modified from https://stackoverflow.com/questions/60637120/detect-circles-in-opencv)
def find_circles(img_cv):
    minDist = 15
    param1 = 50 #500
    param2 = 30 #200 # smaller value-> more false circles
    minRadius = 20
    maxRadius = 40 #10

    # find circles using Hough Circles
    circles = cv2.HoughCircles(img_cv, cv2.HOUGH_GRADIENT, 1, minDist, param1=param1, param2=param2, minRadius=minRadius, maxRadius=maxRadius)
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            cv2.circle(img_cv, (i[0], i[1]), i[2], (255, 255, 255), 2)
    cv2.imshow('img_cv2', img_cv)
    cv2.waitKey(0)
    return circles[0,:,:2]


def main():
    global kps
    global captured
    rospy.init_node('throwertest')

    limb = Limb('right')
    gripper = robot_gripper.Gripper('right_gripper')
    limb.set_joint_position_speed(1.0) # this only sets it for position control - i don't think it matters for us
    limb.set_command_timeout(0.4)

    for j in range(NUM_THROWS):
        # ---------------------------------------------------
        # --- move robot into loading position (for ball) ---
        # ---------------------------------------------------
        _ = input('go to loading pos')
        go_to_joint_angles(-0.5, 0, 0, 0)

        # -----------------
        # --- grab ball ---
        # -----------------
        c = input('to close')
        print("Gripper is_ready:", gripper.is_ready())
        if (c != "n"):  # in case ball is already in gripper
            gripper.open()
            rospy.sleep(1)
        gripper.close()
        rospy.sleep(1)
        print("Gripper force:", gripper.get_force())
        print("Gripper error:", gripper.has_error())
        print("Gripper is_calibrated:", gripper.is_calibrated())
        print("Gripper is_gripping:", gripper.is_gripping())

        # ---------------------------------------
        # --- move to position to detect cups ---
        # ---------------------------------------
        _ = input('find cup')
        kps = []
        captured = False
        go_to_joint_angles(-0.5, -0.785, 0, 0)  # go to detecting pose
        rospy.sleep(3)
        sub = rospy.Subscriber("/io/internal_camera/right_hand_camera/image_rect", Image, capture_img)  # subscribe to right hand camera
        while not captured:  # wait until cups detected by callback function
            rospy.sleep(1)
        while(input('redo? y/N: ') != 'N'):
            kps = []
            captured = False
            while not captured:
                rospy.sleep(1)
        sub.unregister()  # unsubscribe

        print("Cup locations:", kps)
        cup_depth, cup_theta = get_cup_pos(*kps[0]) # return first cup found
        print("Throwing to depth:", cup_depth, "meters and angle", cup_theta)

        # -------------------------------------------
        # --- calculate release pose joint angles ---
        # -------------------------------------------
        _ = input('to calculate')
        wrist_joint = 'right_j5'
        elbow_joint = 'right_j3'
        shoulder_joint = 'right_j1'

        # robot dimensions and max joint velocities (from https://sceweb.sce.uhcl.edu/harman/A_CRS_ROS_SeminarDay3/UNIT3_3_SAWYER/3_3_1_160219_Sawyer_Advanced_Specs_Non_Confidential.pdf)
        base_height = 1.237
        hand_length = 0.3
        wrist_speed = 3.485
        forearm_length = 0.4
        elbow_speed = 1.957
        arm_length = 0.4
        shoulder_speed = 1.328

        # constants for solver
        height = TABLE_HEIGHT + CUP_HEIGHT/393.7
        g = 9.81

        # using pyo nonlinear optimizer with robot constraints
        model = pyo.ConcreteModel()

        # joint angles
        model.theta1 = pyo.Var()
        model.theta3 = pyo.Var()
        model.theta5 = pyo.Var()
        model.dr = pyo.Var() # end effector depth
        model.hr = pyo.Var() # end effector height
        model.vd = pyo.Var() # end effector velocity in the x (depth) direction
        model.vh = pyo.Var() # end effector velocity in the z (height) direction
        model.t = pyo.Var() # time

        # encode robot position and velocity dynamics as constraints
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
        model.Objective = pyo.Objective(expr = (model.dr + model.t*model.vd - cup_depth)**2 + (base_height + model.hr + model.t*model.vh - g*(model.t**2)/2 - height)**2)

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

        # -----------------------------
        # --- move to 'loaded' pose ---
        # -----------------------------

        # calculate 'loaded' pose joint angles based on release pose and throw time
        _ = input('to load')
        theta0_0 = -0.5 - cup_theta  # torso angle based on initial robot angle and cup angle
        theta1_0 = theta1 - RELEASE_TIME * shoulder_speed
        theta3_0 = theta3 - RELEASE_TIME * elbow_speed
        theta5_0 = theta5 - RELEASE_TIME * wrist_speed
        print(f"{theta1_0=}, {theta3_0=}, {theta5_0=}")
        
        # set joint angles to 'loaded' pose
        go_to_joint_angles(theta0_0, theta1_0, theta3_0, theta5_0)
        
        # -----------------------------------------
        # --- throw ball (hopefully into a cup) ---
        # -----------------------------------------
        
        # motion before release
        _ = input('to throw')
        t0 = time.time()
        while not rospy.is_shutdown():
            limb.set_joint_velocities({elbow_joint: elbow_speed, wrist_joint: wrist_speed, shoulder_joint: shoulder_speed})
            d = time.time() - t0
            if d >= RELEASE_TIME:
                break
        
        # open gripper
        gripper.open()
        
        # follow-through motion
        while not rospy.is_shutdown():
            limb.set_joint_velocities({elbow_joint: elbow_speed, wrist_joint: wrist_speed, shoulder_joint: shoulder_speed})
            d = time.time() - t0
            if d >= THROW_TIME:
                break
        print('done!')


if __name__ == '__main__':
    main()
