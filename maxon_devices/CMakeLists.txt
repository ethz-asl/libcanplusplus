cmake_minimum_required(VERSION 2.8.3)
project(maxon_devices)
find_package(catkin REQUIRED COMPONENTS
    libcanplusplus    
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES maxon_devices
  CATKIN_DEPENDS libcanplusplus
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_library(maxon_devices
 src/DeviceEPOS2Motor.cpp
)

target_link_libraries(maxon_devices
  ${catkin_LIBRARIES}
)

