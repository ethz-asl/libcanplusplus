cmake_minimum_required(VERSION 2.6)

################
### INCLUDES ###
################
set(libCAN_INCL ${libCAN_INCL} 
	${CMAKE_CURRENT_SOURCE_DIR} 
PARENT_SCOPE)


###############
### SOURCES ###
###############
set(libCAN_SRCS ${libCAN_SRCS} 
	${CMAKE_CURRENT_SOURCE_DIR}/DeviceEPOS2Motor.cpp
PARENT_SCOPE)

#################
### LIBRARIES ###
#################
set(libCAN_LIBS ${libCAN_LIBS} 
	
PARENT_SCOPE)

