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

set(SOURCES src/main_server.cpp 
            lib/bcm2835/
            lib/camera/
            lib/data_saver/
            lib/imu/
            lib/motors/
            lib/pid/
            lib/temperature/
            lib/tof/
            lib/udp_comm/
            )

add_executable(main_server ${SOURCES})
target_link_libraries( main_server ${Boost_LIBRARIES} ${OpenCV_LIBS})
ament_target_dependencies(main_server rclcpp std_msgs)

install(TARGETS
  main_server
  DESTINATION lib/${PROJECT_NAME})

ament_package()
