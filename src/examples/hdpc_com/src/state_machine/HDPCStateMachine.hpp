/*!
 * @file 	HDPCStateMachine.hpp
 * @brief	State machine for HDPC
 * @author 	Christian Gehring
 * @date 	Apr, 2012
 * @version 1.0
 * @ingroup robotCAN, device
 *
 */

#ifndef HDPCSTATEMACHINE_HPP_
#define HDPCSTATEMACHINE_HPP_


#include "ros/ros.h"
#include "hdpc_com/HDPCStateMachineEnums.h"
#include "hdpc_com/ChangeStateMachine.h"
#include "hdpc_com/Commands.h"
#include "hdpc_com/Readings.h"
#include "hdpc_com/HDPCConst.h"


#include <boost/statechart/event.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/transition.hpp>

#include "BusManager.hpp"

namespace sc = boost::statechart;

//////////////////////////////////////////////////////////////////////////////
/*! events */

//! gets state information, i.e. writes state enum to machine.actualState_
struct EvStateInfo : sc::event< EvStateInfo > {};

//! terminates the state machine
struct EvTerminateSM : sc::event< EvTerminateSM > {};

//! emergency stop -> goes to fault state
struct EvEmergencyStop : sc::event< EvEmergencyStop > {};

//! executes a state
struct EvExecute : sc::event< EvExecute > {};

//! service event stopping
struct EvStopping : sc::event< EvStopping > {};

//! service event reseting
struct EvReseting : sc::event< EvReseting > {};

//! service event starting
struct EvStarting : sc::event< EvStarting > {};

//////////////////////////////////////////////////////////////////////////////

struct StTop;
struct HDPCStateMachine;
//! State machine of the HDPC
struct HDPCStateMachine : sc::state_machine<HDPCStateMachine, StTop>
{
public:


	//! actual state of the state machine
	HDPCSTATE actualState_;

	//! reference to bus manager
	BusManager* busManager_;


	//! commands that are received by commandsCallback()
	hdpc_com::Commands commands_;

	/*! Constructor
	 *
	 * @param busManager
	 * @return
	 */
	HDPCStateMachine(BusManager* busManager);

	//! Destructor
	virtual ~HDPCStateMachine();

	//! initialize ROS stuff
	void initROS();

	//! Read out measurements from CAN message and publish them
	void publishReadings();


private:
	//! Node handle
	boost::shared_ptr<ros::NodeHandle> ros_node_;

	//! ROS service
	ros::ServiceServer ros_service_;

	//! subscriber
	ros::Subscriber ros_sub_;

	//! publisher
	ros::Publisher ros_pub_;

	//! readings that are published
	hdpc_com::Readings readings_;


	/*! ROS service to change the state of the state machine
	 *
	 * @param req	request
	 * @param res	response
	 * @return	true if successful
	 */
	bool srvChangeState(hdpc_com::ChangeStateMachine::Request  &req,
			hdpc_com::ChangeStateMachine::Response &res );

	/*! callback function for commands message
	 * stores the commands in commands_
	 * @param msg	received message
	 */
	void commandsCallback(hdpc_com::Commands msg);


};

#include "HDPCStates.hpp"

#endif /* HDPCSTATEMACHINE_HPP_ */
