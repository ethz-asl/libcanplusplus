cmake_minimum_required(VERSION 2.6)


include_directories(.
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/src/libCAN/SL
	${PROJECT_SOURCE_DIR}/src/libCAN/CANSharedMemory
	${PROJECT_SOURCE_DIR}/src/libCAN/CANMessage
	${PROJECT_SOURCE_DIR}/src/libCAN/Bus
	${PROJECT_SOURCE_DIR}/src/libCAN/Device
	${PROJECT_SOURCE_DIR}/src/libCAN/Device/EPOS2Motor
)
link_directories(${LIBRARY_OUTPUT_PATH})

#MESSAGE(STATUS "${LIBRARY_OUTPUT_PATH}")
set(SRCS
	singleEPOS2and4poleMotorPThread_main.cpp
)

add_executable(singleEPOS2and4poleMotorPThread ${SRCS})
target_link_libraries(singleEPOS2and4poleMotorPThread m bluetooth cpc pthread rt CAN)