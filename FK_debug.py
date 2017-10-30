#!/usr/bin/env python

import numpy as np
from numpy import array
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2, lambdify
from sympy import pprint
from sympy.matrices import Matrix
from sympy.abc import q, a, d, alpha
import logging

logger = logging.getLogger('FK_debug')
logger.setLevel(logging.DEBUG)
logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s.%(msecs)03d %(name)-12s %(levelname)-8s %(message)s',
                    datefmt='%m-%d %H:%M:%S')

logger.debug("start")
# Create symbols for joint variables
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # theta_i

# DH param symbols
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

logger.debug("symbols for joint variables defined")

### KUKA KR210 ###
# DH Parameters
# alpha represents the twist angles
# a represents the link lengths
# d represents the link offsets
# Q represents the joint varbles, the thetas

s = {alpha0:    0,  a0: 0,      d1: 0.75,   q1: q1,
     alpha1: -pi/2, a1: 0.35,   d2: 0,      q2: -pi/2. + q2,
     alpha2:    0,  a2: 1.25,   d3: 0,      q3: q3,
     alpha3: -pi/2, a3: -0.054, d4: 1.50,   q4: q4,
     alpha4:  pi/2, a4: 0,      d5: 0,      q5: q5,
     alpha5: -pi/2, a5: 0,      d6: 0,      q6: q6,
     alpha6:     0, a6: 0,      d7: 0.303, q7: 0
     }
logger.debug("DH Parameters defined")


### Homogenous Transforms

# ht_expr = Matrix([[            cos(q),            -sin(q),            0,              a],
#                [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
#                [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
#                [                  0,                   0,            0,               1]
#                ])
# ht_modules = [{'cos': np.cos, 'sin': np.sin, 'ImmutableDenseMatrix': np.matrix}, 'numpy']
# ht = lambdify((alpha, a, d, q, cos(q), sin(q), cos(alpha), sin(alpha)), ht_expr, ht_modules)
# T0_1 = ht(alpha0, a0, d1, q1, cos(q1), sin(q1), cos(alpha0), sin(alpha0))
# base_link to link1

T0_1 = Matrix([[            cos(q1),            -sin(q1),            0,              a0],
               [sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
               [sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
               [                  0,                   0,            0,               1]
               ])
logger.debug("T0_1 defined")
T0_1 = T0_1.subs(s)
pprint(T0_1)

logger.debug("T0_1 subs")

T1_2 = Matrix([[            cos(q2),            -sin(q2),            0,              a1],
               [sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
               [sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
               [                  0,                   0,            0,               1]
               ])
T1_2 = T1_2.subs(s)
pprint(T1_2)
logger.debug("T1_2 subs")

T2_3 = Matrix([[            cos(q3),            -sin(q3),            0,              a2],
               [sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
               [sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
               [                  0,                   0,            0,               1]
               ])
T2_3 = T2_3.subs(s)
pprint(T2_3)
logger.debug("T2_3 subs")

T3_4 = Matrix([[            cos(q4),            -sin(q4),            0,              a3],
               [sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
               [sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
               [                  0,                   0,            0,               1]
               ])
T3_4 = T3_4.subs(s)
pprint(T3_4)
logger.debug("T3_4 subs")

T4_5 = Matrix([[            cos(q5),            -sin(q5),            0,              a4],
               [sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
               [sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
               [                  0,                   0,            0,               1]
               ])
T4_5 = T4_5.subs(s)
pprint(T4_5)
logger.debug("T4_5 subs")

T5_6 = Matrix([[            cos(q6),            -sin(q6),            0,              a5],
               [sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
               [sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
               [                  0,                   0,            0,               1]
               ])
T5_6 = T5_6.subs(s)
pprint(T5_6)
logger.debug("T5_6 subs")

T6_G = Matrix([[            cos(q7),            -sin(q7),            0,              a6],
               [sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
               [sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
               [                  0,                   0,            0,               1]
               ])
T6_G = T6_G.subs(s)
pprint(T6_G)
logger.debug("T6_G subs")

# Composition of Homogenous Transforms
# T0_2 = simplify(T0_1 * T1_2) # base_link to link_2
# T0_3 = simplify(T0_2 * T2_3) # base_link to link_3
# T0_4 = simplify(T0_3 * T3_4) # base_link to link_4
# T0_5 = simplify(T0_4 * T4_5) # base_link to link_5
# T0_6 = simplify(T0_5 * T5_6) # base_link to link_6
# T0_G = simplify(T0_6 * T6_G) # base_link to gripper
# logger.debug("simplifies done")

T0_2 = T0_1 * T1_2 # base_link to link_2
T0_3 = T0_2 * T2_3 # base_link to link_3
T0_4 = T0_3 * T3_4 # base_link to link_4
T0_5 = T0_4 * T4_5 # base_link to link_5
T0_6 = T0_5 * T5_6 # base_link to link_6
T0_G = T0_6 * T6_G # base_link to gripper
logger.debug("matrix multiplications done")

# Correction Needd to Account of Orientation Difference BEtween Deinition of
# Gripper_link in URDF versus DH Convention
R_z = Matrix([[cos(np.pi), -sin(np.pi), 0, 0],
              [sin(np.pi),  cos(np.pi), 0, 0],
              [         0,           0, 1, 0],
              [         0,           0, 0, 1]
              ])
R_y = Matrix([[ cos(-np.pi/2),  0,  sin(-np.pi/2), 0],
              [             0,  1,              0, 0],
              [-sin(-np.pi/2),  0,  cos(-np.pi/2), 0],
              [             0,  0,              0, 1]
              ])
# R_corr = simplify(R_z * R_y)
R_corr = R_z * R_y
logger.debug("R_corr ")

#### Numerically evalute transforms (compare this with output of tf_echo!)
joints={q1: 3.19, q2:0.75, q3: -2.77, q4: -2.14, q5: -0.33, q6:-0.04}
# joints={q1:0, q2:0, q3: 0, q4: 0, q5: 0, q6:0}

print("T0_1 = ", T0_1.evalf(subs=joints))
logger.debug("T0_1")
print("T0_2 = ", T0_2.evalf(subs=joints))
logger.debug("T0_2")
print("T0_3 = ", T0_3.evalf(subs=joints))
logger.debug("T0_3")
print("T0_4 = ", T0_4.evalf(subs=joints))
logger.debug("T0_4")
print("T0_5 = ", T0_5.evalf(subs=joints))
logger.debug("T0_5")
print("T0_6 = ", T0_6.evalf(subs=joints))
logger.debug("T0_6")
print("T0_G = ", T0_G.evalf(subs=joints))
logger.debug("T0_G")

# Total Homogenous Transforms Between Base_link and Gripper_link with
# Orientation Correction Applied
# T_total = simplify(T0_G * R_corr)
# logger.debug("T_total simplify")
T_total = T0_G * R_corr
logger.debug("T_total matrix multiply")
# pprint(T_total)
print("T_total = ", T_total.evalf(subs=joints))

# logger.debug("final simplify start")
# pprint(simplify(T_total))

logger.debug("finish")
