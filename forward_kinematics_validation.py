import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
np.set_printoptions(precision=3, suppress=True)

def dh_transform(theta, d, a, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha),  np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0,              np.sin(alpha),                 np.cos(alpha),                 d],
        [0,              0,                             0,                             1]
    ])


def compute_jacobian(dh_params):
    n = len(dh_params)  # Number of joints
    T = sp.eye(4)  # Initial transformation matrix
    transforms = [T]  # Store transformations for each joint
    
    # Compute forward kinematics symbolically
    for d, a, alpha, theta in dh_params:
        T = T @ dh_transform(theta, d, a, alpha)
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

def plot_robot(dh_params):
    """
    Plot a robot using DH parameters, ensuring intermediate points are plotted when d > 0.
    
    :param dh_params: List of DH parameters (theta, d, a, alpha)
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Starting position
    origin = np.array([0, 0, 0, 1])
    points = [origin[:3]]

    T = np.eye(4)  # Initial transformation matrix (identity)
    i=0
    for d,a,alpha,theta in dh_params:
        # Handle translation along z-axis if d > 0
        if d != 0:
            intermediate_T = T @ dh_transform(0, d, 0, 0)
            points.append(intermediate_T[:3, 3])  # Add intermediate point for translation

        print("Transformation Matrix from frame ",i, " to ", i+1, ":")
        print(dh_transform(theta, d, a, alpha))
        print("------------------------------------------------------")
        print("------------------------------------------------------")
        i=i+1
        # Calculate the transformation matrix for the current joint
        T = T @ dh_transform(theta, d, a, alpha)
        points.append(T[:3, 3])  # Add the end of this link
    points = np.array(points)
    print("Transformation Matrix from frame 0 to 6 for all theta=0")
    print(T)
    print("------------------------------------------------------")
    print("------------------------------------------------------")

    print("End Effector Position Validation: ", points[-1])
    ax.plot(points[:, 0], points[:, 1], points[:, 2], '-o', label="Robot Structure")
    ax.set_xlim([-0.75,0.75])
    ax.set_ylim([-0.75,0.75])
    ax.set_zlim([0,1.5])
    # Configure the plot
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Robot Plot Using DH Parameters')
    ax.legend()
    plt.show()

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

# Plot the robot
plot_robot(dh_params)
