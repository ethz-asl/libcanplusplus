/*!
 * @file 	HDPCStateMachine.cpp
 * @brief	State machine for HDPC
 * @author 	Christian Gehring
 * @date 	Apr, 2012
 * @version 1.0
 * @ingroup robotCAN, device
 *
 */
#include "HDPCStateMachine.hpp"

#include "DeviceELMOBaseMotor.hpp"
#include "DeviceELMODrivingMotor.hpp"
#include "DeviceELMOSteeringMotor.hpp"

HDPCStateMachine::HDPCStateMachine(BusManager* busManager):busManager_(busManager),isInStFault_(false),isStarting_(false)
{
	//DeviceManager* devices = busManager_->getBus(0)->getDeviceManager();
	for (int iDevice=0; iDevice < commands_.command.size(); iDevice++) {
		commands_.isActive[iDevice] = false;
		commands_.command[iDevice] = 0.0;
	}
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

	DeviceManager* devices = busManager_->getBus(0)->getDeviceManager();
	for (int iDevice=0; iDevice < devices->getSize(); iDevice++) {
		DeviceELMOBaseMotor* motor =  (DeviceELMOBaseMotor*) devices->getDevice(iDevice);
		motor->printStatusword();
	}

	return true;
}

void HDPCStateMachine::commandsCallback(hdpc_com::Commands msg)
{
	commands_ = msg;
}

void HDPCStateMachine::publishReadings()
{

	DeviceManager* devices = busManager_->getBus(0)->getDeviceManager();
	for (int iDevice=0; iDevice < devices->getSize(); iDevice++) {
		DeviceELMOBaseMotor* motor =  (DeviceELMOBaseMotor*) devices->getDevice(iDevice);
		readings_.position[iDevice] = motor->getPosition();
		readings_.velocity[iDevice] = motor->getVelocity();
		//readings_.analog[iDevice] = motor->getAnalog();
		readings_.analog[iDevice] = motor->getAbsJointPosition();
		readings_.current[iDevice] = motor->getCurrent();
		//motor->printStatusword();
	}

	/* publish readings */
	ros_pub_.publish(readings_);
}

void HDPCStateMachine::checkStatus()
{
	DeviceManager* devices = busManager_->getBus(0)->getDeviceManager();
	for (int iDevice=0; iDevice < devices->getSize(); iDevice++) {
		DeviceELMOBaseMotor* motor =  (DeviceELMOBaseMotor*) devices->getDevice(iDevice);
		if (motor->getStatus()->isFault()) {
			process_event(EvEmergencyStop());
		}
		if (motor->getStatus()->isDisabled()) {
			process_event(EvStopping());
		}
	}
}
