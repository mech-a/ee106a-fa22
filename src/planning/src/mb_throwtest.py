#!/usr/bin/env python
import rospy
import time
import pyomo.environ as pyo
import math

from intera_interface import gripper as robot_gripper
from intera_interface import Limb

# -0.5 0.0 0.0 0.0 0.0 -2.96 1.57 is a neutral position to begin throwing from only J5.

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
    height = 1
    g = 9.81

    model = pyo.ConcreteModel()
    model.theta1 = pyo.Var()
    model.theta3 = pyo.Var()
    model.theta5 = pyo.Var()
    model.dr = pyo.Var()
    model.hr = pyo.Var()
    model.vd = pyo.Var()
    model.vh = pyo.Var()
    model.t = pyo.Var()

    model.Constraint1 = pyo.Constraint(
        expr = model.vd == (arm_length * pyo.cos(math.pi/2 + model.theta1) * shoulder_speed) +
                           (forearm_length * pyo.cos(math.pi/2 + model.theta1 + model.theta3) * (shoulder_speed + elbow_speed)) +
                           (hand_length * pyo.cos(math.pi/2 + model.theta1 + model.theta3 +  model.theta5) * (shoulder_speed + elbow_speed + wrist_speed))
                           )
    model.Constraint2 = pyo.Constraint(
        expr = model.dr == (arm_length * pyo.sin(math.pi/2 + model.theta1)) +
                           (forearm_length * pyo.sin(math.pi/2 + model.theta1 + model.theta3)) +
                           (hand_length * pyo.sin(math.pi/2 + model.theta1 + model.theta3 +  model.theta5)) 
                           )
    model.Constraint3 = pyo.Constraint(
        expr = model.hr == (arm_length * pyo.cos(math.pi/2 + model.theta1)) +
                           (forearm_length * pyo.cos(math.pi/2 + model.theta1 + model.theta3)) +
                           (hand_length * pyo.cos(math.pi/2 + model.theta1 + model.theta3 +  model.theta5)) 
                           )
    model.Constraint4 = pyo.Constraint(
        expr = model.vh == -((arm_length * pyo.sin(math.pi/2 + model.theta1) * shoulder_speed) +
                           (forearm_length * pyo.sin(math.pi/2 + model.theta1 + model.theta3) * (shoulder_speed + elbow_speed)) +
                           (hand_length * pyo.sin(math.pi/2 + model.theta1 + model.theta3 +  model.theta5) * (shoulder_speed + elbow_speed + wrist_speed))
                           ))
    # model.Constraint4 = pyo.Constraint(
    #     expr = depth == model.dr + (model.vd * pyo.cos(model.theta1 + model.theta3 +  model.theta5) + 
    #                            pyo.sqrt((model.vd * pyo.sin(-(model.theta1 + model.theta3 +  model.theta5)))**2 + 2*g*(base_height + model.hr - height)))/g)
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

    _ = input('to throw')
    total_time = 0.4
    throw_time = 0.3
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
