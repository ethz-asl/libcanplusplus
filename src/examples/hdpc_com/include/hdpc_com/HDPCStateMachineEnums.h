/*!
 * @file 	HDPCStateMachineEnums.h
 * @brief	Enumerations used in the state machine of HDPC
 * @author 	Christian Gehring
 * @date 	Apr, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 */
#ifndef HDPCSTATEMACHINEENUMS_H_
#define HDPCSTATEMACHINEENUMS_H_

//! States that are returned by srvChangeState()
enum HDPCSTATE {
	//! is initializing
	SM_INIT = 0,
	//! stop mode
	SM_STOP,
	//! drive mode
	SM_DRIVE,
	//! an error happened
	SM_FAULT
};

//! Events that are allowed to be received by srvChangeState()
enum HDPCEVENT {
	//! does not change any state, but returns the actual state
	EVENT_READ_STATE = 0,
	//! starts the motors, i.e. enables the motors and goes to drive state
	EVENT_START,
	//! stops the motors, i.e. disables the motors and goes to stop state
	EVENT_STOP,
	//! resets / re-initializes the motors in init state and goes then to stop state
	EVENT_RESET
};


#endif /* HDPCSTATEMACHINEENUMS_H_ */
