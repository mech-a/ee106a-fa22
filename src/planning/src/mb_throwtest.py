#!/usr/bin/env python
import rospy
import time
import pyomo.environ as pyo
import math

from intera_interface import gripper as robot_gripper
from intera_interface import Limb

# -0.5 -0.9 0.0 -1.4 0.0 -2.0 1.57 is a neutral position to begin throwing from only J5.

def main():
    rospy.init_node('throwertest')
    limb = Limb('right')
    gripper = robot_gripper.Gripper('right_gripper')
    limb.set_joint_position_speed(1.0) # this only sets it for position control - i don't think it matters for us
    limb.set_command_timeout(0.4)

    _ = input('to calibrate')
    gripper.calibrate()

    _ = input('to close')
    gripper.close()

    _ = input('to calculate')
    wrist_joint = 'right_j5'
    elbow_joint = 'right_j3'
    shoulder_joint = 'right_j1'

    base_height = 1.237
    hand_length = 0.25
    wrist_speed = 3.485
    forearm_length = 0.4
    elbow_speed = 1.957
    arm_length = 0.4
    shoulder_speed = 1.328

    depth = 1.7
    height = 0.729
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

    _ = input('to throw')
    t0 = time.time()
    while not rospy.is_shutdown():
        limb.set_joint_velocities({elbow_joint: elbow_speed, wrist_joint: wrist_speed, shoulder_joint: shoulder_speed})
        d = time.time() - t0
        if d >= throw_time:
            break
    gripper.open()
    while not rospy.is_shutdown():
        limb.set_joint_velocities({elbow_joint: elbow_speed, wrist_joint: wrist_speed, shoulder_joint: shoulder_speed})
        d = time.time() - t0
        if d >= total_time:
            break
    print('done!')

if __name__ == '__main__':
    main()
