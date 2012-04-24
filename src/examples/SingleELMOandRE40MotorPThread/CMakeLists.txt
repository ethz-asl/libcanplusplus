cmake_minimum_required(VERSION 2.6)

set(CAN++_MODULE_PATH   "~/libcanplusplus/trunk/cmake")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CAN++_MODULE_PATH})	
find_package(CAN++ REQUIRED)

include_directories(.
	${CAN++_INCLUDE_DIRS}

)
link_directories(${LIBRARY_OUTPUT_PATH})

#MESSAGE(STATUS "${LIBRARY_OUTPUT_PATH}")
set(SRCS
	singleELMOandRE40MotorPThread_main.cpp
)

add_executable(singleELMOandRE40MotorPThread ${SRCS})
target_link_libraries(singleELMOandRE40MotorPThread ${CAN++_LIBRARIES} cpc)

