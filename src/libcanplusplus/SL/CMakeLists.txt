cmake_minimum_required(VERSION 2.6)

################
### INCLUDES ###
################
set(libCAN_INCL ${libCAN_INCL} 
	${CMAKE_CURRENT_SOURCE_DIR} 
PARENT_SCOPE)


if (USE_SL_FUNCTIONS)
	set(libCAN_INCL ${libCAN_INCL} 
		${CMAKE_CURRENT_SOURCE_DIR} 
		$ENV{LAB_ROOT}/include
		$ENV{LAB_ROOT}/starleth/include
	PARENT_SCOPE)
else (USE_SL_FUNCTIONS)
	set(libCAN_INCL ${libCAN_INCL} 
		${CMAKE_CURRENT_SOURCE_DIR} 
	PARENT_SCOPE)
endif (USE_SL_FUNCTIONS)

###############
### SOURCES ###
###############
if (USE_SL_FUNCTIONS)
set(libCAN_SRCS ${libCAN_SRCS} 
	${CMAKE_CURRENT_SOURCE_DIR}/SLSharedMemory.c
PARENT_SCOPE)
else (USE_SL_FUNCTIONS)

if (COMPILE_XENOMAI)
	set(libCAN_SRCS ${libCAN_SRCS} 
		${CMAKE_CURRENT_SOURCE_DIR}/SLVx2XenoWrappers.c
		${CMAKE_CURRENT_SOURCE_DIR}/SLSharedMemory.c
		${CMAKE_CURRENT_SOURCE_DIR}/SLUtility.c 
	PARENT_SCOPE)
else (COMPILE_XENOMAI)
	set(libCAN_SRCS ${libCAN_SRCS} 
		${CMAKE_CURRENT_SOURCE_DIR}/SLVx2UnixWrappers.c
		${CMAKE_CURRENT_SOURCE_DIR}/SLSharedMemory.c
		${CMAKE_CURRENT_SOURCE_DIR}/SLUtility.c 
	PARENT_SCOPE)	
endif (COMPILE_XENOMAI)

endif (USE_SL_FUNCTIONS)

#################
### LIBRARIES ###
#################
set(libCAN_LIBS ${libCAN_LIBS} 
	
PARENT_SCOPE)

