#!/bin/bash

# Source ROS 2 setup file (adjust path as per your installation)
source /opt/ros/humble/setup.bash
source /home/gursel/ros2_ws/install/setup.bash


# Command 1: Static Transform Publisher
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map odom &

# Wait for the static transform publisher to initialize (adjust sleep time as needed)
sleep 3

# Command 2: Map Server
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/gursel/ros2_ws/src/diff_robot/map/my_map.yaml &

# Wait for the map server to initialize (adjust sleep time as needed)
sleep 4

# Command 3: Bring up Map Server Lifecycle
ros2 run nav2_util lifecycle_bringup map_server &
echo "MAP DATA PUBLISHING.."

# Wait for the map server lifecycle node to initialize (adjust sleep time as needed)
sleep 5

# Command 4: AMCL
ros2 run nav2_amcl amcl &

# Wait for AMCL to initialize (adjust sleep time as needed)
sleep 4

# Command 5: Bring up AMCL Lifecycle
ros2 run nav2_util lifecycle_bringup amcl &

# Wait for AMCL lifecycle node to initialize (adjust sleep time as needed)
sleep 4

# Command 6: Python A* Path Finding Script
python3 /home/gursel/ros2_ws/src/diff_robot/planning/a_star_path_finding.py &
echo "A* PATH PLANNER"

# sleep 4

# python3 /home/gursel/ros2_ws/src/diff_robot/control/visualize.py

