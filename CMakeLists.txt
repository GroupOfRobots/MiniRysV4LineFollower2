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

set ( Motors_LIBRARIES libmotors.so )
link_directories( RobotLineFollower/robot/libs/motors/build )
include_directories(RobotLineFollower/robot/libs/motors/include)
set ( Camera_LIBRARIES libcamera.so )
link_directories( RobotLineFollower/robot/libs/camera/build )
include_directories(RobotLineFollower/robot/libs/camera/include)
set ( Udp_Communication_LIBRARIES libudp_comm.so )
link_directories( RobotLineFollower/robot/libs/udp_comm/build )
include_directories(RobotLineFollower/robot/libs/udp_comm/include)
set ( Pid_LIBRARIES libpid.so )
link_directories( RobotLineFollower/robot/libs/pid/build )
include_directories(RobotLineFollower/robot/libs/pid/include)
set ( Data_Saver_LIBRARIES libdata_saver.so )
link_directories( RobotLineFollower/robot/libs/data_saver/build )
include_directories(RobotLineFollower/robot/libs/data_saver/include)


add_executable(main_server src/main_server.cpp)
target_link_libraries( main_server ${Boost_LIBRARIES} ${OpenCV_LIBS} ${Motors_LIBRARIES} ${Camera_LIBRARIES} ${Udp_Communication_LIBRARIES} ${Pid_LIBRARIES} ${Data_Saver_LIBRARIES})

ament_target_dependencies(main_server rclcpp std_msgs)

install(TARGETS
  main_server
  DESTINATION lib/${PROJECT_NAME})

ament_package()
