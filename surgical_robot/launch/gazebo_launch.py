from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from osrf_pycommon.terminal_color import ansi
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Paths to URDF and World files
    urdf_file_name = "surgical_robot.urdf.xacro"  # Change to your URDF filename
    urdf = os.path.join(
        get_package_share_directory("surgical_robot"), "urdf", urdf_file_name
    )
    world_file_name = "empty_world.world"  # Change to your custom world file name
    world = os.path.join(
        get_package_share_directory("surgical_robot"), "worlds", world_file_name
    )

    # Set the path to your custom model directory
    custom_model_path = os.path.join(
        get_package_share_directory("surgical_robot"), "models"
    )
    # Path to the controller YAML file
    controller_config = os.path.join(
        get_package_share_directory("surgical_robot"),  
        "config",
        "control.yaml",  
    )

    # Add the custom model path to GAZEBO_MODEL_PATH
    os.environ["GAZEBO_MODEL_PATH"] = custom_model_path

    # Launch description
    ld = LaunchDescription()

    # Launch Arguments
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time if true",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "robot_urdf",
            default_value=urdf,
            description="surgical_robot/urdf/surgical_robot.urdf.xacro",
        )
    )

    # Gazebo server launch with the custom world
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gzserver.launch.py",
            )
        ),
        launch_arguments={
            "world": world,
            "extra_gazebo_args": "-s libgazebo_ros_factory.so",  # Load the Gazebo ROS Factory plugin
        }.items(),
    )

    # Gazebo client launch
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gzclient.launch.py",
            )
        )
    )

    # Controller Manager Node
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        output="screen",
    )

    # Robot State Publisher node to publish URDF to the /robot_description topic
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        arguments=[LaunchConfiguration("robot_urdf")],
    )

    # Joint State Broadcaster spawner
    spawner_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawner_robot_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller"],  # Replace with your controller name
        output="screen",
        parameters=[controller_config],  # Pass the YAML file as a parameter
    )

    # Spawn entity node to spawn the URDF model into the Gazebo simulation
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "surgical_robot",  # Replace with your robot's name
            "-file",
            LaunchConfiguration("robot_urdf"),
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.0",
        ],  # Position to spawn the robot in the world
        output="screen",
    )

    # Add actions to the launch description
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(robot_state_publisher)
    ld.add_action(controller_manager_node)

    # Add a small delay before spawning the robot to ensure Gazebo is ready
    ld.add_action(
        TimerAction(
            period=1.0,  # Delay (seconds) to allow Gazebo to initialize
            actions=[spawn_entity],
        )
    )

    # Add controller and broadcaster spawners (these depend on the robot being spawned)
    ld.add_action(
        TimerAction(
            period=5.0,  # Further delay to ensure the robot is spawned before spawners are launched
            actions=[spawner_joint_state_broadcaster, spawner_robot_controller],
        )
    )

    return ld
