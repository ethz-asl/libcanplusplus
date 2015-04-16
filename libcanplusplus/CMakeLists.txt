cmake_minimum_required(VERSION 2.6)

project(libcanplusplus)

find_package(catkin REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES libcanplusplus
  CATKIN_DEPENDS 
#  DEPENDS system_lib
)

add_definitions(-Wall )
add_definitions(-fPIC)


find_package(Boost REQUIRED)


include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

if(COMPILE_XENOMAI)
	set(libCAN_INCL ${libCAN_INCL}
		/usr/include/xenomai
	PARENT_SCOPE)
endif(COMPILE_XENOMAI)


add_library(libcanplusplus 
    src/Bus.cpp
    src/BusManager.cpp
    src/CANOpenMsg.cpp
    src/PDOManager.cpp
    src/SDOManager.cpp
    src/SDOMsg.cpp
    src/SDOReadMsg.cpp
    src/SDOWriteMsg.cpp
    src/Device.cpp
    src/DeviceManager.cpp
    src/CanManagerBase.cpp
)
target_link_libraries(libcanplusplus
    ${catkin_LIBRARIES}
)


