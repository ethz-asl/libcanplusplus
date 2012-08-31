/*!
 * @file 	CANSharedMemoryDecl.hpp
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 */

#ifndef CAN_HPP_
#define CAN_HPP_

#include "LibCANConfig.h"


typedef struct {
	char flag;
	char rtr;
	int COBId;
	unsigned char length;
	unsigned char value[8];
} CAN_BusDataMeas;

typedef struct {
	unsigned char flag;
	unsigned char rtr;
	int COBId;
	unsigned char length;
	unsigned char value[8];
} CAN_BusDataDes;


#endif /* CAN_HPP_ */
