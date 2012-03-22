/*!
 * @file 	SLVariables.c
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 */
#ifndef CANVARIABLES_HPP_
#define CANVARIABLES_HPP_


#include "CANSharedMemoryDecl.hpp"


#ifdef __cplusplus
extern "C" {
#endif


//! string to define part of the name of a shared variable
char* robot_name = "libCAN";
int parent_process_id = 0;
//! Required by shared memory functions
int servo_base_rate = 400;

#ifdef __cplusplus
}
#endif



#endif /* CANVARIABLES_HPP_ */
