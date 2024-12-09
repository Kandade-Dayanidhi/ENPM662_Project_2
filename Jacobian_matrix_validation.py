import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def dh_transform(d, a, alpha, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha),  np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0,              np.sin(alpha),                 np.cos(alpha),                 d],
        [0,              0,                             0,                             1]
    ])


def compute_jacobian(dh_params):
    n = len(dh_params)  # Number of joints
    T = np.eye(4)  # Initial transformation matrix
    transforms = [T]  # Store transformations for each joint
    
    # Compute forward kinematics symbolically
    for d, a, alpha, theta in dh_params:
        T = T @ dh_transform(d, a, alpha, theta)
        transforms.append(T)
    
    # Compute Jacobian symbolically
    J = np.zeros((6, n))  # 6xN matrix (linear and angular velocities)
    o_n = transforms[-1][:3, 3]  # Position of the end-effector
    
    for i in range(n):
        T_i = transforms[i]  # Transformation up to the i-th joint
        z_i = T_i[:3, 2]  # z-axis of the i-th joint
        o_i = T_i[:3, 3]  # Position of the i-th joint
        
        # Linear velocity component
        J[:3, i] = np.cross(z_i, o_n - o_i)
        # Angular velocity component
        J[3:, i] = z_i
    return J

# Example DH parameters: [(theta, d, a, alpha), ...]
theta1=0
theta2=0
theta3=0
theta4=0
theta5=0
theta6=0

dh_params = np.array([
    [0.1273, 0,      np.pi/2, np.pi+theta1],  # Base to first joint
    [0.17,      -0.612, 0,      -np.pi/2+theta2],  # First to second joint
    [-0.17,      -0.5723, 0,      0+theta3],  # Second to third joint
    [0.1639, 0,      np.pi/2, -np.pi/2 + theta4],  # Third to fourth joint
    [0.1157, 0,     -np.pi/2, 0+theta5],  # Fourth to fifth joint
    [0.0922, 0,      0,      0+theta6]   # Fifth to end-effector
])
J=compute_jacobian(dh_params)
# Plot the robot
print("Jacobian Matrix:")
print(np.round(J,4))
print("------------------------------------------------------")
print("------------------------------------------------------")