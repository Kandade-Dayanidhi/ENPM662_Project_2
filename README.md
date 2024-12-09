# ENPM662_Project_2

Steps to run the package

1. Copy the packages into the src folder inside the workspace folder.
2. Build the workspace
3. source it "source insatll/setup.bash"
4. Run the launch file by running "ros2 launch surgical_robot gazebo_launch.py"
5. open another tab and source ros2 and repeat the step 3
6. Before running the node, please install "pip install visual-kinematics" and repeat the steps from 2-3.
7. run the node for controlling the manipulator by running "ros2 run py_pubsub joint_cmd"

optional

8. To visualize it in Rviz, please run "ros2 run surgical_robot display.launch.py"
