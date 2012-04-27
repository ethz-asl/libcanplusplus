cmake_minimum_required(VERSION 2.6)

set(CAN++_MODULE_PATH   "${PATH_TO_LIBRARY}/cmake")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CAN++_MODULE_PATH})	
find_package(CAN++ REQUIRED)

INCLUDE_DIRECTORIES(.
	${CAN++_INCLUDE_DIRS}
)
link_directories(${LIBRARY_OUTPUT_PATH} )


set(SRCS
	singleEPOS2and4poleMotorPThread_main.cpp

)

add_executable(singleEPOS2and4poleMotorPThread ${SRCS})
target_link_libraries(singleEPOS2and4poleMotorPThread ${CAN++_LIBRARIES} cpc)