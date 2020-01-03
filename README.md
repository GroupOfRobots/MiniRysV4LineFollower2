# robot_line_follower
Package of line follower functionality under ROS2 environment.

## Running package on robot: 
1. `cd <your_ws>` 
2. `colcon build` 
3. `source install/setup.bash` 
4. `ros2 launch robot_line_follower robot_line_follower.launch.py` - for test
5. `ros2 launch robot_line_follower data_pub_example.launch.py` - to run developed version 

## Running package on PC: 
1. `cd <your_ws>` 
2. `colcon build` 
3. `source install/setup.bash` 
4. `ros2 run robot_line_follower client` - for decoding image sent from robot in order to visualize it in rviz2

## Parameters
All parameters can be changed in files in `yaml` directory.

## Requirements
To use this package following package is required:
* https://index.ros.org/p/cv_bridge/github-ros-perception-vision_opencv/
