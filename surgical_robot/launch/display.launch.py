from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
import os
import launch_ros.descriptions
import launch
import xacro

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Data Input
    xacro_file = "surgical_robot.urdf.xacro"
    package_description = "surgical_robot"
    pkg_share = launch_ros.substitutions.FindPackageShare(package=package_description).find(package_description)

    # Parse URDF/XACRO file
    robot_desc_path = os.path.join(get_package_share_directory(
        package_description), "urdf", xacro_file)
    robot_desc = xacro.process_file(robot_desc_path)
    xml = robot_desc.toxml()

    # RViz Configuration
    rviz_config_dir = PathJoinSubstitution(
        [FindPackageShare("surgical_robot"), "rviz", "display_default.rviz"]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )

    # Robot State Publisher
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': xml}],
        output="screen"
    )

    joint_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Static TF Transform
    tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['1', '0', '0', '0', '0', '0', '1', '/map', '/dummy_link'],
    )

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                             description='Flag to enable use_sim_time'),
        robot_state_publisher,
        rviz_node,
        joint_state_gui,
        tf,
    ])

