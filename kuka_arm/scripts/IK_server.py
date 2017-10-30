#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1

    # Your FK code here

    # Create symbols
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # theta_i
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

	# Create Modified DH parameters
    # alpha represents the twist angles
    # a represents the link lengths
    # d represents the link offsets
    # Q represents the joint varbles, the thetas
    s = {alpha0:    0,  a0: 0,      d1: 0.75,
         alpha1: -pi/2, a1: 0.35,   d2: 0,      q2: -pi/2. + q2,
         alpha2:    0,  a2: 1.25,   d3: 0,
         alpha3: -pi/2, a3: -0.054, d4: 1.50,
         alpha4:  pi/2, a4: 0,      d5: 0,
         alpha5: -pi/2, a5: 0,      d6: 0,
         alpha6:     0, a6: 0,      d7: 0.303, q7: 0
         }

	# Create individual transformation matrices
    T0_1 = Matrix([[            cos(q1),            -sin(q1),            0,              a0],
                    [sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                    [sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                    [                  0,                   0,            0,               1]
                    ])
    T0_1 = T0_1.subs(s)

    T1_2 = Matrix([[            cos(q2),            -sin(q2),            0,              a1],
                    [sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                    [sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                    [                  0,                   0,            0,               1]
                    ])
    T1_2 = T1_2.subs(s)

    T2_3 = Matrix([[            cos(q3),            -sin(q3),            0,              a2],
                    [sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                    [sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                    [                  0,                   0,            0,               1]
                    ])
    T2_3 = T2_3.subs(s)

    T3_4 = Matrix([[            cos(q4),            -sin(q4),            0,              a3],
                    [sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                    [sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                    [                  0,                   0,            0,               1]
                    ])
    T3_4 = T3_4.subs(s)

    T4_5 = Matrix([[            cos(q5),            -sin(q5),            0,              a4],
                    [sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                    [sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                    [                  0,                   0,            0,               1]
                    ])
    T4_5 = T4_5.subs(s)

    T5_6 = Matrix([[            cos(q6),            -sin(q6),            0,              a5],
                    [sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                    [sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                    [                  0,                   0,            0,               1]
                    ])
    T5_6 = T5_6.subs(s)

    T6_G = Matrix([[            cos(q7),            -sin(q7),            0,              a6],
                    [sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                    [sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                    [                  0,                   0,            0,               1]
                    ])
    T6_G = T6_G.subs(s)
    # Calculate Gripper/end-effector matrix
    T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

    # Extract rotation matrices from the transformation matrices
    # Define RPY rotation matrices
    # http://planning.cs.uiuc.edu/node102.html
    r, p, y = symbols('r p y')

    ROT_x = Matrix([[1, 0,0],
                    [0, cos(r), -sin(r)],
                    [0, sin(r), cos(r)]
                    ])  # ROLL
    ROT_y = Matrix([[cos(p), 0, sin(p)],
                    [0, 1, 0],
                    [-sin(p), 0, cos(p)]
                    ])  # PITCH
    ROT_z = Matrix([[cos(y), -sin(y), 0],
                    [sin(y), cos(y), 0],
                    [0, 0, 1]
                    ])  # YAW

    # Gripper_link in URDF versus DH Convention
    # R_z = Matrix([[cos(pi), -sin(pi), 0, 0],
    #               [sin(pi),  cos(pi), 0, 0],
    #               [      0,         0, 1, 0],
    #               [      0,         0, 0, 1]
    #               ])
    # R_y = Matrix([[ cos(-pi/2.), 0, sin(-pi/2.), 0],
    #               [           0, 1,              0, 0],
    #               [-sin(-pi/2.), 0,  cos(-pi/2.), 0],
    #               [           0, 0,               0, 1]
    #               ])
    # # R_corr = simplify(R_z * R_y)
    # R_corr = R_z * R_y

    # Initialize service response
    joint_trajectory_list = []
    for x in xrange(0, len(req.poses)):
        # IK code starts here
        joint_trajectory_point = JointTrajectoryPoint()

        # Extract end-effector position and orientation from request
        # px,py,pz = end-effector position
        # roll, pitch, yaw = end-effector orientation
        px = req.poses[x].position.x
        py = req.poses[x].position.y
        pz = req.poses[x].position.z

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [req.poses[x].orientation.x, req.poses[x].orientation.y,
                req.poses[x].orientation.z, req.poses[x].orientation.w])

        # Your IK code here
        # Compensate for rotation discrepancy between DH parameters and Gazebo
        # Calculate joint angles using Geometric IK method
        ###
        Rot_Error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))

        ROT_EE = ROT_z * ROT_y * ROT_x * Rot_Error

        Rrpy = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})


        # Calculate joint angles using Geometric IK method
        ###
        # Wrist Center
        nx = Rrpy[0,2]
        ny = Rrpy[1,2]
        nz = Rrpy[2,2]
        l = s[d7]  # End Effector length
        wcx = px - (s[d6] + l) * nx
        wcy = py - (s[d6] + l) * ny
        wcz = pz - (s[d6] + l) * nz

        # Calculate joint angles using Geometrix IK method
        # More information can be found in the Inverse Kinematics with Kuka KR210
        def clip(n, min, max):
            if n < min:
                n = min
            elif n > max:
                n = max
            return n


        theta1 = clip(atan2(wcy, wcx), radians(-185), radians(185))

        # SSS triangle for theta2 and theta3
        side_a = 1.501
        side_b = sqrt(pow((sqrt(wcx * wcx + wcy * wcy) - 0.35), 2) + pow((wcz - 0.75), 2))
        side_c = 1.25

        side_a_sq = side_a * side_a
        side_b_sq = side_b * side_b
        side_c_sq = side_c * side_c

        angle_a = acos((side_b_sq + side_c_sq - side_a_sq)/(2 * side_b * side_c))
        angle_b = acos((side_a_sq + side_c_sq - side_b_sq)/(2 * side_a * side_c))
        angle_c = acos((side_a_sq + side_b_sq - side_c_sq)/(2 * side_a * side_b))

        theta2 = pi/2 - angle_a - atan2(wcz - 0.75, sqrt(wcx * wcx + wcy * wcy) - 0.35)
        theta2 = clip(theta2, radians(-45), radians(85))
        theta3 = pi/2 - (angle_b + 0.036)  # 0.036 accounts for sag in link4 of -0.054 m
        theta3 = clip(theta3, radians(-210), radians(155-90))

        R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
        R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

        # R3_6 = R0_3.inv("LU") * Rrpy
        R3_6 = R0_3.transpose() * Rrpy

        # Euler angles from rotation matrix
        # More informaiton can be found in hte Euler Angles from a Rotation Matrix section
        theta4 = clip(atan2(R3_6[2,2], -R3_6[0,2]), radians(-350), radians(350))
        theta5 = clip(atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2]), radians(-125), radians(125))
        theta6 = clip(atan2(-R3_6[1,2], R3_6[1,0]), radians(-350), radians(350))

        # Populate response for the IK request
        # In the next line replace theta1,theta2...,theta6 by your joint angle variables
        joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
        joint_trajectory_list.append(joint_trajectory_point)

    rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
    return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
