# - Try to find CAN++
# Once done, this will define
#
#  CAN++_FOUND - system has CAN++
#  CAN++_INCLUDE_DIRS - the CAN++ include directories
#  CAN++_LIBRARIES - link these to use CAN++



set(CAN++_INCLUDE_DIRS_ROOT "${CAN++_MODULE_PATH}/..")
set(CAN++_INCLUDE_DIRS
	${CAN++_INCLUDE_DIRS_ROOT}/include
	${CAN++_INCLUDE_DIRS_ROOT}/src/libcanplusplus/SL
	${CAN++_INCLUDE_DIRS_ROOT}/src/libcanplusplus/CANSharedMemory
	${CAN++_INCLUDE_DIRS_ROOT}/src/libcanplusplus/CANMessage
	${CAN++_INCLUDE_DIRS_ROOT}/src/libcanplusplus/bus
	${CAN++_INCLUDE_DIRS_ROOT}/src/libcanplusplus/devices
	${CAN++_INCLUDE_DIRS_ROOT}/src/libcanplusplus/devices/EPOS2Motor
        ${CAN++_INCLUDE_DIRS_ROOT}/src/libcanplusplus/devices/ELMOMotor
)

set(CAN++_LIBRARIES
  m bluetooth pthread rt CAN++
)

link_directories("${CAN++_INCLUDE_DIRS_ROOT}/lib")

set(CAN++_FOUND TRUE)