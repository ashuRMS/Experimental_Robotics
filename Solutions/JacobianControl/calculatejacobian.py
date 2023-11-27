#!/usr/bin/env python3
import numpy as np
import sympy as sp
from sympy import Matrix, N
import math

def forward_kinematics_inv_Jacobian(q):         #return ee_pos, np.linalg.pinv(Jv)
    # Function for forward kinematics and inverse Jacobian calculation
    
    epsilon = sp.Matrix([1, 1, 1, 1])
    n = 4

    # Define symbolic variables
    theta = sp.symbols('theta[0:%d]' % n)
    alpha = np.array([0, math.pi/2, 0, 0])
    a = np.array([0.012, 0, 0.130, 0.124])
    d = np.array([0.077, 0, 0, 0])

    # Initialize the transformation matrices
    Ttemp = sp.eye(4)
    HTM = [sp.eye(4) for _ in range(n)]

    for i in range(n):
        # Calculate the transformation matrix elements using symbolic variables
        t11 = sp.cos(theta[i])
        t12 = -sp.sin(theta[i])
        t13 = 0
        t14 = a[i]

        t21 = sp.sin(theta[i]) * sp.cos(alpha[i])
        t22 = sp.cos(theta[i]) * sp.cos(alpha[i])
        t23 = -sp.sin(alpha[i])
        t24 = -d[i] * sp.sin(alpha[i])

        t31 = sp.sin(theta[i]) * sp.sin(alpha[i])
        t32 = sp.cos(theta[i]) * sp.sin(alpha[i])
        t33 = sp.cos(alpha[i])
        t34 = d[i] * sp.cos(alpha[i])

        Tiim1 = sp.Matrix([[t11, t12, t13, t14],
                           [t21, t22, t23, t24],
                           [t31, t32, t33, t34],
                           [0, 0, 0, 1]])

        Ti0 = Ttemp * Tiim1
        HTM[i] = Ti0
        Ttemp = Ti0

    # Homogeneous Transformation matrix
    Tn0 = HTM[n - 1]
    HTM_list = [HTM[0], HTM[1], HTM[2], HTM[3]]

    # End effector's position
    pE0_h = Tn0 * sp.Matrix([0.126, 0, 0, 1])
    pE0 = sp.Matrix(pE0_h[:3])

    # Initialize the Jacobian matrix
    Jv = sp.Matrix.ones(3, n)

    for i, item in enumerate(HTM_list):
        v1 = epsilon[i] * sp.Matrix(item[:3, 2])
        v2 = pE0 - sp.Matrix(item[:3, 3])
        cross_product = v1.cross(v2)
        Jv[:, i] = cross_product

    # Substitute joint values into the Jacobian matrix
    theta_values = [math.radians(q[0]), math.radians(q[1]) + math.pi/2 - 0.1853, math.radians(q[2]) - math.pi/2 + 0.1853, math.radians(q[3])]
    Jv_substituted = Jv.subs({theta[0]: theta_values[0], theta[1]: theta_values[1],
                              theta[2]: theta_values[2], theta[3]: theta_values[3]})
    Jv_substituted = N(Jv_substituted, 3)
    Jv_substituted = Jv_substituted.applyfunc(lambda x: round(x, 3))

    # Substitute joint values into end effector position
    ee_pos = pE0.subs({theta[0]: theta_values[0], theta[1]: theta_values[1],
                      theta[2]: theta_values[2], theta[3]: theta_values[3]})
    Jv = np.array(Jv_substituted, dtype=float)

    # Display the Jacobian matrix
    print("Jacobian Matrix:\n")
    sp.pprint(Jv, use_unicode=True)

    return Jv

# Get user input for joint angles in radians
q_user = [float(input(f"Enter joint angle {i+1} in degrees: ")) for i in range(4)]

# Calculate and display the Jacobian
Jv= forward_kinematics_inv_Jacobian(q_user)

