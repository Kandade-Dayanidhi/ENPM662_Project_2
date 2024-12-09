import sympy as sp
import numpy as np

def dh_transform_symbolic(theta, d, a, alpha):
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta) * sp.cos(alpha),  sp.sin(theta) * sp.sin(alpha), a * sp.cos(theta)],
        [sp.sin(theta),  sp.cos(theta) * sp.cos(alpha), -sp.cos(theta) * sp.sin(alpha), a * sp.sin(theta)],
        [0,              sp.sin(alpha),                 sp.cos(alpha),                 d],
        [0,              0,                             0,                             1]
    ])

def compute_jacobian_symbolic(dh_params):
    n = len(dh_params)  # Number of joints
    T = sp.eye(4)  # Initial transformation matrix
    transforms = [T]  # Store transformations for each joint
    
    # Compute forward kinematics symbolically
    for d, a, alpha, theta in dh_params:
        T = T @ dh_transform_symbolic(theta, d, a, alpha)
        transforms.append(T)
    
    # Compute Jacobian symbolically
    J = sp.zeros(6, n)  # 6xN matrix (linear and angular velocities)
    o_n = transforms[-1][:3, 3]  # Position of the end-effector
    
    for i in range(n):
        T_i = transforms[i]  # Transformation up to the i-th joint
        z_i = T_i[:3, 2]  # z-axis of the i-th joint
        o_i = T_i[:3, 3]  # Position of the i-th joint
        
        # Linear velocity component
        J[:3, i] = z_i.cross(o_n - o_i)
        # Angular velocity component
        J[3:, i] = z_i
    
    return sp.simplify(J)

# Define symbolic DH parameters
theta1, theta2, theta3, theta4, theta5 = sp.symbols('theta1 theta2 theta3 theta4 theta5')

dh_params = np.array([
    [0.1273, 0,      np.pi/2, np.pi+ theta1],  # Base to first joint
    [0.1639,      -0.612, 0,      -np.pi/2 + theta2],  # First to second joint
    [-0.1639,      -0.5723, 0,      0 + theta3],  # Second to third joint
    [0.1639, 0,      np.pi/2, -np.pi/2 + + theta4],  # Third to fourth joint
    [0.1157, 0,     -np.pi/2, 0 + + theta5],  # Fourth to fifth joint
    [0.0922, 0,      0,      0]   # Fifth to end-effector
])
# Compute the symbolic Jacobian
J_symbolic = compute_jacobian_symbolic(dh_params)
sp.pprint(J_symbolic)
