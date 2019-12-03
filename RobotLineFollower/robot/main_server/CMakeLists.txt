cmake_minimum_required(VERSION 3.1)
project( main_server )
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
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
link_directories( ../libs/motors/build )
include_directories(../libs/motors/include)
set ( Camera_LIBRARIES libcamera.so )
link_directories( ../libs/camera/build )
include_directories(../libs/camera/include)
set ( Udp_Communication_LIBRARIES libudp_comm.so )
link_directories( ../libs/udp_comm/build )
include_directories(../libs/udp_comm/include)
set ( Pid_LIBRARIES libpid.so )
link_directories( ../libs/pid/build )
include_directories(../libs/pid/include)
set ( Data_Saver_LIBRARIES libdata_saver.so )
link_directories( ../libs/data_saver/build )
include_directories(../libs/data_saver/include)

add_executable( main_server src/main_server.cpp)
target_link_libraries( main_server ${Boost_LIBRARIES} ${OpenCV_LIBS} ${Motors_LIBRARIES} ${Camera_LIBRARIES} ${Udp_Communication_LIBRARIES} ${Pid_LIBRARIES} ${Data_Saver_LIBRARIES})





