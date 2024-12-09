#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from visual_kinematics.RobotSerial import RobotSerial, Frame
from visual_kinematics.RobotTrajectory import RobotTrajectory
import numpy as np
from math import pi


class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        np.set_printoptions(precision=3, suppress=True)
        # Publishers
        self.command_publisher = self.create_publisher(
            Float64MultiArray, '/position_controller/commands', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.publish_joint_data)  # Publish at 10 Hz

        # DH Parameters
        self.dh_params = np.array([
            [0.1273, 0,      np.pi/2, np.pi],  # Base to first joint
            [0,      -0.612, 0,      -np.pi/2],  # First to second joint
            [0,      -0.5723, 0,      0],  # Second to third joint
            [0.1639, 0,      np.pi/2, -np.pi/2],  # Third to fourth joint
            [0.1157, 0,     -np.pi/2, 0],  # Fourth to fifth joint
            [0.0922, 0,      0,      0]   # Fifth to end-effector
        ])
        self.robot = RobotSerial(self.dh_params)

        # Trajectory
        self.trajectory = self.generate_trajectory()
        self.trajectory_time_step = 0.01
        self.trajectory_time = 0.0

    def generate_trajectory(self):
        """Generate the trajectory for the robot."""
        # Initial pose
        theta = np.array([0., 0., 0., 0., 0., 0.])
        f = self.robot.forward(theta)
        print("-------forward-------")
        print("end frame t_4_4:")
        print(f.t_4_4)
        print("end frame xyz:")
        print(f.t_3_1)           

        print("end frame abc:")
        print(f.euler_3)

        frames = [Frame.from_euler_3(f.euler_3, np.array(f.t_3_1)),
                  Frame.from_euler_3([-0.25,0.0,-2.99], np.array([[-0.5], [-0.8], [0.3]])),
                  Frame.from_euler_3([0.25,0.0,-2.99], np.array([[-0.5], [0.8], [0.3]])),               
                  Frame.from_euler_3([-0.25,0.0,-2.99], np.array([[0.3], [-0.8], [0.3]])),
                  Frame.from_euler_3([0.25,0.0,-2.99], np.array([[0.3], [0.8], [0.3]])),
                  Frame.from_euler_3([-0.25,0.0,-2.99], np.array([[-0.25], [-0.8], [0.3]])),
                  Frame.from_euler_3([0.25,0.0,-2.99], np.array([[1.0], [-0.15], [0.3]])),
                  Frame.from_euler_3([-0.25,0.0,-2.99], np.array([[0.6], [-0.8], [0.3]])),
                  Frame.from_euler_3([0.25,0.0,-2.99], np.array([[1.0], [0.3], [0.3]])),
                  Frame.from_euler_3([-0.25,0.0,-2.99], np.array([[0.0], [-0.8], [0.3]])),
                  Frame.from_euler_3([0.25,0.0,-2.99], np.array([[-1.0], [0.0], [0.3]])),
                  
                  ]
        time_points = np.array([0.0, 0.5,1.0,1.5,2.0,2.5,3.0,3.5,4.0,4.5,5.0])
        # Create trajectory
        trajectory = RobotTrajectory(self.robot, frames, time_points)
        trajectory.show(motion="p2p")
        # Interpolate joint values and time points
        num_segments = 10000  # Define how finely the trajectory is divided
        self.inter_values, self.inter_time_points = trajectory.interpolate(
                10000, motion="p2p", method="linear"
            )
        return trajectory

    def publish_joint_data(self):
        """Publish joint angles and states."""
        # Check if the trajectory is complete
        if self.trajectory_time >= self.trajectory.time_points[-1]:
            self.get_logger().info("Trajectory complete!")
            self.trajectory_time = 0.0


        print(self.inter_values[-1])
        # Find the closest time point in the interpolated data
        closest_index = np.searchsorted(self.inter_time_points, self.trajectory_time)
        if closest_index >= len(self.inter_values):
            self.get_logger().warn("Trajectory index out of range!")
            return

        joint_angles = self.inter_values[closest_index].tolist()
        self.trajectory_time += self.trajectory_time_step
        joint_angles[1]=-joint_angles[1]
        joint_angles[2]=-joint_angles[2]
        joint_angles[4]=-joint_angles[4]
        # Publish to /position_controller/commands
        commands_msg = Float64MultiArray()
        commands_msg.data = joint_angles
        print(self.trajectory_time, closest_index, joint_angles)
        self.command_publisher.publish(commands_msg)






def main(args=None):
    rclpy.init(args=args)
    node = JointPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()