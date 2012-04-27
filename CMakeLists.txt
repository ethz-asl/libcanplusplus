cmake_minimum_required(VERSION 2.6)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(libCAN_INCL
	${PROJECT_SOURCE_DIR}/include
)

OPTION(COMPILE_XENOMAI "Turn on to compile with xenomai libraries" OFF )
OPTION(USE_SL_FUNCTIONS "Turn on if it is used within SL" OFF )
OPTION(COMPILE_EXAMPLES "Turn on to compile examples" ON )
OPTION(PATH_TO_LIBRARY "Turn on to compile examples" "~/libcanplusplus/trunk" )

####################
## SUBDIRECTORIES ##
####################
add_subdirectory(src/libcanplusplus)
add_subdirectory(src/examples)