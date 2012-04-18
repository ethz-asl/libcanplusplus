/*
 * HDPCStateMachine.cpp
 *
 *  Created on: Apr 15, 2012
 *      Author: gech
 */

#include "HDPCStateMachine.hpp"

#include "DeviceELMOBaseMotor.hpp"
#include "DeviceELMODrivingMotor.hpp"
#include "DeviceELMOSteeringMotor.hpp"

HDPCStateMachine::HDPCStateMachine(BusManager* busManager):busManager_(busManager)
{

}

HDPCStateMachine::~HDPCStateMachine()
{

}



void HDPCStateMachine::initROS()
{
	ros_node_.reset(new ros::NodeHandle("~"));
	ros_service_ = ros_node_->advertiseService("changeState", &HDPCStateMachine::srvChangeState, this);
	ros_sub_ = ros_node_->subscribe("commands", 1, &HDPCStateMachine::commandsCallback, this);
	ros_pub_ = ros_node_->advertise<hdpc_com::Readings>("readings", 1);
}


bool HDPCStateMachine::srvChangeState(hdpc_com::ChangeStateMachine::Request  &req,
		hdpc_com::ChangeStateMachine::Response &res )
{
	/* process event */
	switch(req.event) {
	case EVENT_READSTATEONLY:
		/* do nothing */
		break;
	case EVENT_STOPPING:
		process_event(EvStopping());
		break;
	case EVENT_STARTING:
		process_event(EvStarting());
		break;
	case EVENT_RESETING:
		process_event(EvReseting());
		break;
	default:
		;
	}

	/* get state */
	process_event(EvStateInfo());
	res.state = (int) actualState_;

	ROS_INFO("Service Change State: ");
	ROS_INFO("event: %d", req.event);
	ROS_INFO("state: %d", actualState_);

	return true;
}

void HDPCStateMachine::commandsCallback(const hdpc_com::Commands::ConstPtr& msg)
{

}

void HDPCStateMachine::publishReadings()
{

	DeviceManager* devices = busManager_->getBus(0)->getDeviceManager();
	for (int iDevice=0; iDevice < devices->getSize(); iDevice++) {
		DeviceELMOBaseMotor* motor =  (DeviceELMOBaseMotor*) devices->getDevice(iDevice);
		readings_.position[iDevice] = motor->getPosition();
		readings_.velocity[iDevice] = motor->getVelocity();
		readings_.analog[iDevice] = motor->getAnalog();
		readings_.current[iDevice] = motor->getCurrent();
	}

	/* publish readings */
	ros_pub_.publish(readings_);
}
