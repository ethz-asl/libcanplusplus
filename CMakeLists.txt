cmake_minimum_required(VERSION 2.6)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(libCAN_INCL
	${PROJECT_SOURCE_DIR}/include
)

OPTION(COMPILE_XENOMAI "Turn on to compile with xenomai libraries" OFF )
OPTION(USE_SL_FUNCTIONS "Turn on if it is used within SL" OFF )

####################
## SUBDIRECTORIES ##
####################
add_subdirectory(src/libcanplusplus)
add_subdirectory(src/examples)