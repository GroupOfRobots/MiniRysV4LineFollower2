# RobotCamera

This repo contains line detection algorithm for miniRys robot. 

## Running program on Raspberry PI: 
1. sudo modprobe bcm2835-v4l2 
2. chmod +x build_libs.sh 
3. ./build_libs.sh 
4. cd main_server
5. mkdir build  
6. cd build  
7. cmake ..  
8. make    
9. ./main_server 
  
## Requirements for Raspberry PI
1. Boost (at least v 1.58)  
2. OpenCv (v 2.4.9.1)  
3. modprobe bcm2835-v4l2 driver  
  
## Useful links  
https://www.pyimagesearch.com/2018/09/26/install-opencv-4-on-your-raspberry-pi/  
https://www.learnopencv.com/install-opencv-4-on-raspberry-pi/    
http://answers.opencv.org/question/78820/opencv-python-raspberry-pi-minimum-required-size-on-disk/  
https://www.raspberrypi.org/documentation/linux/usage/users.md  
https://www.raspberrypi.org/forums/viewtopic.php?t=55504

## Quick OpenCv installation on Raspberry Pi       
command: sudo apt-get install libopencv-dev   
  


