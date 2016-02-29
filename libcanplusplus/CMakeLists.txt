cmake_minimum_required(VERSION 2.6)
project(libcanplusplus)

find_package(catkin REQUIRED)
find_package(Boost REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES libcanplusplus
#  CATKIN_DEPENDS 
  DEPENDS Boost
)

###########
## Build ##
###########
add_definitions(-Wall )
add_definitions(-fPIC)
add_definitions(-std=c++11)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

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
)
target_link_libraries(libcanplusplus
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

