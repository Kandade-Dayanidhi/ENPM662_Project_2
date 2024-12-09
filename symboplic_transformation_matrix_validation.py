import sympy as sp
import numpy as np

def dh_transform_symbolic(theta, d, a, alpha):

    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta) * sp.cos(alpha),  sp.sin(theta) * sp.sin(alpha), a * sp.cos(theta)],
        [sp.sin(theta),  sp.cos(theta) * sp.cos(alpha), -sp.cos(theta) * sp.sin(alpha), a * sp.sin(theta)],
        [0,              sp.sin(alpha),                 sp.cos(alpha),                 d],
        [0,              0,                             0,                             1]
    ])

def compute_full_transformation(dh_params):
    """
    Compute the symbolic transformation matrix from the first frame to the last frame.
    
    :param dh_params: List of DH parameters [(theta, d, a, alpha), ...], where theta is symbolic
    :return: Symbolic 4x4 transformation matrix
    """
    T = sp.eye(4)  # Initialize as the identity matrix
    for d, a, alpha, theta in dh_params:
        T = T @ dh_transform_symbolic(theta, d, a, alpha)  # Multiply transformation matrices
    return sp.simplify(T)

# Define symbolic variables for thetas
theta1, theta2, theta3, theta4, theta5 = sp.symbols('theta1 theta2 theta3 theta4 theta5')

# Example DH parameters with symbolic thetas

dh_params = np.array([
    [0.1273, 0,      np.pi/2, np.pi+theta1],  # Base to first joint
    [0.1639,      -0.612, 0,      -np.pi/2+theta2],  # First to second joint
    [-0.1639,      -0.5723, 0,      0+theta3],  # Second to third joint
    [0.1639, 0,      np.pi/2, -np.pi/2+theta4],  # Third to fourth joint
    [0.1157, 0,     -np.pi/2, 0+theta5],  # Fourth to fifth joint
    [0.0922, 0,      0,      0]   # Fifth to end-effector
])


# Compute the full transformation matrix
T_final = compute_full_transformation(dh_params)

# Display the result
sp.pprint(T_final)
