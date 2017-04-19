cmake_minimum_required(VERSION 2.6)

####################
## SUBDIRECTORIES ##
####################
if (USE_SL_FUNCTIONS)
else (USE_SL_FUNCTIONS)

if (COMPILE_EXAMPLES)
add_subdirectory(SingleEPOS2and4poleMotorPThread)
add_subdirectory(SingleELMOandRE40MotorPThread)
endif (COMPILE_EXAMPLES)

endif (USE_SL_FUNCTIONS)
