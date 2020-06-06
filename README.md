# robot_line_follower
Package of line follower functionality under ROS2 environment. Tested on Raspberry Pi 4B with Ubuntu Bionic 18.04 from: https://jamesachambers.com/raspberry-pi-4-ubuntu-server-desktop-18-04-3-image-unofficial

## Running package on robot: 
1. `cd <your_ws>` 
2. `colcon build` 
3. `source install/setup.bash` 
4. `ros2 launch robot_line_follower rys_line_follower.launch.py` 

## Running package on PC: 
1. `cd <your_ws>` 
2. `colcon build` 
3. `source install/setup.bash` 
4. `ros2 run robot_line_follower client` - for decoding image sent from robot in order to visualize it in rviz2

## Parameters
All parameters can be changed in files in `yaml` directory.

## Requirements
* ROS2 Dashing Diademata
* ROS2 cv_bridge package - https://index.ros.org/p/cv_bridge/github-ros-perception-vision_opencv/
* Boost (at least v 1.58)  
* OpenCV 3 - for robot
* bcm2835-v4l2 driver - for robot camera
* python smbus module - for temperature node

## FAQ's
* How to quickly install OpenCV on Raspberry Pi? Run `sudo apt-get install libopencv-dev`.
* How to quickly install Boost on Raspberry Pi? Run `sudo apt-get install libboost-all-dev`.
* How to quickly install ROS2 on Raspberry Pi with Ubuntu Bionic 18.04? See https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/.
* How to install python smbus module? Run `sudo apt-get install python3-smbus`. 
