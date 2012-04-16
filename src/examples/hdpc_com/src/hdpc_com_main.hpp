/*
 * hdpc_com_main.hpp
 *
 *  Created on: Apr 15, 2012
 *      Author: gech
 */

#ifndef HDPC_COM_MAIN_HPP_
#define HDPC_COM_MAIN_HPP_


/* Set CAN bitrate to 1Mbps */
#define BTR0 0x00
#define BTR1 0x14
/* Set CAN bitrate to 500kbps */
//#define BTR0 0x00
//#define BTR1 0x1c


//! struct of bus routine initial arguments
struct BusRoutineArguments {
	// identifier of the bus [0,3]
	int iBus;
	// handle of CAN bus
	int handle;
	// time step in ms
	double time_step_ms;
};


//! RxPDO index of RxPDOManager in busManager
enum RxPDOs {
	RxPDO_SYNC = 0,
	RxPDO_MOTOR = 1,
};

//! TxPDO index of TxPDOManager in busManager
enum TxPDOs {
	TxPDO_MOTOR,
};

//! shared memory ID of sent messages
enum DESSMID {
	DESSMID_RxPDO_SYNC = 1,
	DESSMID_RxPDO_MOTOR = 0,
	DESSMID_SDO = 2,
};

//! shared memory ID of received messages
enum MEASSMID {
	MEASSMID_TxPDO_MOTOR = 0,
	MEASSMID_SDO_MOTOR = 1,
	MEASSMID_TxPDO_ANALOG_CURRENT = 2
};

//! CAN Node ID given by the DIP switches
enum CANNODEID {
	NODEID_MOTOR=1,
};


#endif /* HDPC_COM_MAIN_HPP_ */
