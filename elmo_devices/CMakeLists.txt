cmake_minimum_required(VERSION 2.8.3)
project(elmo_devices)
find_package(catkin REQUIRED COMPONENTS
    libcanplusplus    
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES elmo_devices
  CATKIN_DEPENDS libcanplusplus
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_library(elmo_devices
 src/DeviceELMOMotor.cpp
)

target_link_libraries(elmo_devices
  ${catkin_LIBRARIES}
)

