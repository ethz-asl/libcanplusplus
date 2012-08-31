cmake_minimum_required(VERSION 2.6)

#project(CAN++)

SET(LIBNAME CAN++)	 #Name of the library that will be generated


# Set Build Type
#   * None (CMAKE_C_FLAGS or CMAKE_CXX_FLAGS used)
#   * Debug (CMAKE_C_FLAGS_DEBUG or CMAKE_CXX_FLAGS_DEBUG)
#   * Release (CMAKE_C_FLAGS_RELEASE or CMAKE_CXX_FLAGS_RELEASE)
#   * RelWithDebInfo (CMAKE_C_FLAGS_RELWITHDEBINFO or CMAKE_CXX_FLAGS_RELWITHDEBINFO
#   * MinSizeRel (CMAKE_C_FLAGS_MINSIZEREL or CMAKE_CXX_FLAGS_MINSIZEREL) 
SET(CMAKE_BUILD_TYPE RelWithDebInfo)

add_definitions(-Wall )




# configure a header file to pass some of the CMake settings
# to the source code
configure_file (
  "${CMAKE_CURRENT_SOURCE_DIR}/LibCANConfig.h.in"
  "${PROJECT_SOURCE_DIR}/include/LibCANConfig.h"
)
  #"${PROJECT_SOURCE_DIR}/include/LibCANConfig.h"
find_package(Boost REQUIRED)


################
### INCLUDES ###
################
set(libCAN_INCL ${libCAN_INCL} 
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/include
PARENT_SCOPE)

if(COMPILE_XENOMAI)
	set(libCAN_INCL ${libCAN_INCL}
		/usr/include/xenomai
	PARENT_SCOPE)
endif(COMPILE_XENOMAI)


####################
## SUBDIRECTORIES ##
####################
add_subdirectory(bus)
add_subdirectory(CANMessage)
add_subdirectory(CANSharedMemory)
add_subdirectory(SL)
add_subdirectory(devices)


include_directories(${libCAN_INCL})

add_library(${LIBNAME} STATIC ${libCAN_SRCS})

if (USE_SL_FUNCTIONS)
	install (TARGETS ${LIBNAME} DESTINATION $ENV{LAB_ROOT}/lib/$ENV{MACHTYPE})
endif (USE_SL_FUNCTIONS)


