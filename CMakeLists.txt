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
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package( OpenCV REQUIRED )
find_package(cv_bridge REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
find_package( Boost COMPONENTS system thread REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
include_directories(include ${cv_bridge_INCLUDE_DIRS})

ament_python_install_package(lib/tmp)

set(msg_files 
  "msg/ProcessData.msg"
  )

rosidl_generate_interfaces(${PROJECT_NAME}
   ${msg_files}
   DEPENDENCIES std_msgs
   DEPENDENCIES sensor_msgs
)

set(SOURCES_MAIN src/main.cpp 
            lib/camera/ContourFinding.cpp
            lib/detector/Detector.cpp
            lib/controller/Controller.cpp
            lib/data_saver/DataSaver.cpp
            lib/data_publisher/DataPublisher.cpp
            lib/bcm2835/bcm2835.c
            lib/motors/l6470.cpp
            lib/motors/l6470commands.cpp
            lib/motors/l6470config.cpp
            lib/motors/l6470dump.cpp
            lib/motors/l6470support.cpp
            lib/motors/motors.cpp
            lib/pid/Pid.cpp
            )

set(SOURCES_CLIENT src/client.cpp)

add_executable(main ${SOURCES_MAIN})
target_link_libraries(main ${Boost_LIBRARIES} ${OpenCV_LIBS})
ament_target_dependencies(main rclcpp sensor_msgs cv_bridge)
rosidl_target_interfaces(main ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(client ${SOURCES_CLIENT})
target_link_libraries(client ${OpenCV_LIBS} ${Boost_LIBRARIES})
ament_target_dependencies(client rclcpp sensor_msgs)

install(TARGETS
  client
  main
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  lib/tmp/tmp102.py
  lib/tmp/tmp_node.py
  DESTINATION lib/${PROJECT_NAME})

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
