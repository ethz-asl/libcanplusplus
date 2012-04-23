cmake_minimum_required(VERSION 2.6)


include_directories(.
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/src/libcanplusplus/SL
	${PROJECT_SOURCE_DIR}/src/libcanplusplus/CANSharedMemory
	${PROJECT_SOURCE_DIR}/src/libcanplusplus/CANMessage
	${PROJECT_SOURCE_DIR}/src/libcanplusplus/bus
	${PROJECT_SOURCE_DIR}/src/libcanplusplus/devices
	${PROJECT_SOURCE_DIR}/src/libcanplusplus/devices/ELMOMotor

)
link_directories(${LIBRARY_OUTPUT_PATH})

#MESSAGE(STATUS "${LIBRARY_OUTPUT_PATH}")
set(SRCS
	singleELMOandRE40MotorPThread_main.cpp
)

add_executable(singleELMOandRE40MotorPThread ${SRCS})
target_link_libraries(singleELMOandRE40MotorPThread m bluetooth cpc pthread rt CAN)

