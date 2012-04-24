cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

rosbuild_add_boost_directories()

# Set path to libcanplusplus/trunk/cmake
set(CAN++_MODULE_PATH   "../../cmake")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CAN++_MODULE_PATH})	
find_package(CAN++ REQUIRED)

include_directories(
	${CAN++_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}/.
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/src/devices
	${PROJECT_SOURCE_DIR}/src/state_machine
	
)

set(SRCS
	./src/hdpc_com_main.cpp
	./src/devices/DeviceELMOBaseMotor.cpp
	./src/devices/DeviceELMOSteeringMotor.cpp
	./src/devices/DeviceELMODrivingMotor.cpp
	./src/state_machine/HDPCStateMachine.cpp
	./src/state_machine/HDPCStates.cpp
)

rosbuild_add_executable(hdpc_com ${SRCS})
target_link_libraries(hdpc_com ${CAN++_LIBRARIES} cpc ) 
