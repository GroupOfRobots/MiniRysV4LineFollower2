cmake_minimum_required(VERSION 3.5)
project(robot_line_follower)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
find_package( Boost COMPONENTS system thread REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

file(READ "/etc/issue" ETC_ISSUE)
string(REGEX MATCH "Raspbian|Ubuntu" DIST ${ETC_ISSUE})

if(DIST STREQUAL "Raspbian")
    message(STATUS ">>>> Found Raspbian <<<<")

elseif(DIST STREQUAL "Ubuntu")
    message(STATUS ">>>> Found Ubuntu <<<<")

else()
    message(STATUS ">>>> Found unknown distribution <<<<")

endif()

ament_python_install_package(lib/tmp/)

set(SOURCES src/main.cpp 
            lib/bcm2835/bcm2835.c
            lib/camera/CenterFinding.cpp
            lib/camera/ContourFinding.cpp
            lib/data_saver/DataSaver.cpp
            lib/imu/LSM6DS3.cpp
            lib/imu/filter.cpp
            lib/motors/l6470.cpp
            lib/motors/l6470commands.cpp
            lib/motors/l6470config.cpp
            lib/motors/l6470dump.cpp
            lib/motors/l6470support.cpp
            lib/motors/motors.cpp
            lib/pid/Pid.cpp
            lib/tof/VL53L1X.cpp
            lib/udp_comm/UdpJpgFrameStreamer.cpp
            )

add_executable(main_server ${SOURCES})
target_link_libraries( main_server ${Boost_LIBRARIES} ${OpenCV_LIBS})
ament_target_dependencies(main_server rclcpp std_msgs)

install(TARGETS
  main_server
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  lib/tmp/tmp_node.py
  DESTINATION bin)

ament_package()
