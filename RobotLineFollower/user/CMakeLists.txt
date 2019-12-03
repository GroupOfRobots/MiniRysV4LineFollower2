
# to run with parameter type:
# 1. cmake .. -DSERVER = 1
# to run without parameter type:
# 1. cmake ..

cmake_minimum_required(VERSION 3.1)
# Enable C++11
project( test1 )
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
#set(OpenCV_DIR OpenCV_Home_Dir/installation/OpenCV-master/lib/cmake/opencv4)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

FIND_PACKAGE( Boost COMPONENTS system thread REQUIRED)
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

add_executable( client src/client.cpp )
target_link_libraries( client ${Boost_LIBRARIES} ${OpenCV_LIBS})

