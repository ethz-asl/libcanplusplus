/*!
 * @file 	HDPCStates.cpp
 * @brief	States of the state machine for HDPC
 * @author 	Christian Gehring
 * @date 	Apr, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 */
#include "hdpc_com/HDPCStateMachineEnums.h"
#include "HDPCStateMachine.hpp"

#include "DeviceELMOBaseMotor.hpp"
#include "DeviceELMODrivingMotor.hpp"
#include "DeviceELMOSteeringMotor.hpp"


#include <iostream>

//////////////////////////////////////////////////////////////////////////////
StTop::StTop( my_context ctx ) :
  my_base( ctx )
{
	ROS_INFO("Entering StTop");
}

StTop::~StTop()
{
	ROS_INFO("Exiting StTop");
}

sc::result StTop::react( const EvExecute& )
{
	return discard_event();
}

sc::result StTop::react( const EvEmergencyStop& )
{

	return transit<StFault>();
}

sc::result StTop::react( const EvTerminateSM& )
{
	return terminate();
}


//////////////////////////////////////////////////////////////////////////////
StInit::StInit( my_context ctx ) :
  my_base( ctx ), iDevice_(0), waitForDeviceCount_(10000)
{
	ROS_INFO("Entering StInit");
	outermost_context_type & machine = outermost_context();
	machine.busManager_->getBus(0)->getRxPDOManager()->setSending(false);

}

StInit::~StInit() {
	ROS_INFO("Exiting StInit");
	outermost_context_type & machine = outermost_context();
	machine.busManager_->getBus(0)->getRxPDOManager()->setSending(true);

}

sc::result StInit::react( const EvExecute& )
{
	outermost_context_type & machine = outermost_context();

	/* The previous device should have some time to initialize
	 * counts ~= 2x no. of SDOs to init
	 */
	if (waitForDeviceCount_++ > 100) {
		waitForDeviceCount_ = 0;

		DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();

		/* check if all devices are initialized */
		if (iDevice_ >= devices->getSize()) {
			return transit<StHoming>();
		}

		ROS_INFO("Init device %d", iDevice_);
		/* initialize device */

		DeviceELMOBaseMotor* motor =  (DeviceELMOBaseMotor*) devices->getDevice(iDevice_);
		motor->initDevice();

		iDevice_++;
	}

	return forward_event();
}


sc::result StInit::react( const EvStateInfo& )
{
	outermost_context().actualState_ = SM_INIT;
	ROS_INFO("StInit");
	return discard_event();
}
//////////////////////////////////////////////////////////////////////////////
StHoming::StHoming( my_context ctx ) :
  my_base( ctx ),waitCount_(0)
{
	ROS_INFO("Entering StHoming");
	outermost_context_type & machine = outermost_context();
	DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();

	for (int iDevice=6; iDevice < 10; iDevice++) {
		DeviceELMOSteeringMotor* motor =  (DeviceELMOSteeringMotor*) devices->getDevice(iDevice);
		motor->absCurrentJointPosition_ = 0.0;
		motor->setHomeOffsetJointPosition(0.0);
	}
}

StHoming::~StHoming() {
	ROS_INFO("Exiting StHoming");
}

sc::result StHoming::react( const EvExecute& )
{
	outermost_context_type & machine = outermost_context();
	DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();

	const int nSamples = 200;
	if (waitCount_ < nSamples) {
		for (int iDevice=6; iDevice < 10; iDevice++) {
			DeviceELMOSteeringMotor* motor =  (DeviceELMOSteeringMotor*) devices->getDevice(iDevice);
			motor->absCurrentJointPosition_ += motor->getAbsJointPosition();
		}
	} else if (waitCount_ == nSamples){
		for (int iDevice=6; iDevice < 10; iDevice++) {
			DeviceELMOSteeringMotor* motor =  (DeviceELMOSteeringMotor*) devices->getDevice(iDevice);
			motor->absCurrentJointPosition_ /= nSamples;
			//ROS_INFO("Device%d: pos=%lf",iDevice,motor->absCurrentJointPosition_);
			const double homeOffsetJointPosition_rad = motor->absCurrentJointPosition_ - motor->getPosition();
			motor->setHomeOffsetJointPosition(homeOffsetJointPosition_rad);
			motor->setPositionLimits(motor->getDeviceParams()->positionLimits);
		}
	} else if (waitCount_ > nSamples+10) {
		return transit<StStop>();
	}
	waitCount_++;


	return forward_event();
}


sc::result StHoming::react( const EvStateInfo& )
{
	outermost_context().actualState_ = SM_INIT;
	ROS_INFO("StHoming");
	return discard_event();
}


//////////////////////////////////////////////////////////////////////////////
StStop::StStop( my_context ctx ) :
  my_base( ctx )
{
	ROS_INFO("Entering StStop");
	outermost_context_type & machine = outermost_context();
	DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();
	for (int iDevice=0; iDevice < devices->getSize(); iDevice++) {
		DeviceELMOBaseMotor* motor =  (DeviceELMOBaseMotor*) devices->getDevice(iDevice);
		motor->setDisableMotor();

		machine.commands_.isActive[iDevice] = true;
        machine.commands_.velocity[iDevice] = 0.0;
        machine.commands_.position[iDevice] = motor->getPosition();
	}



}

StStop::~StStop() {
	ROS_INFO("Exiting StStop");
}

sc::result StStop::react( const EvExecute& )
{
	return discard_event();
}


sc::result StStop::react( const EvStateInfo& )
{
	outermost_context().actualState_ = SM_STOP;
	ROS_INFO("StStop");
	return discard_event();
}

sc::result StStop::react( const EvStopping& )
{
	return discard_event();
}

//////////////////////////////////////////////////////////////////////////////
StFault::StFault( my_context ctx ) :
  my_base( ctx )
{
	ROS_INFO("Entering StFault");
	outermost_context_type & machine = outermost_context();
	DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();
	for (int iDevice=0; iDevice < devices->getSize(); iDevice++) {
		DeviceELMOBaseMotor* motor =  (DeviceELMOBaseMotor*) devices->getDevice(iDevice);
		motor->setDisableMotor();

		machine.commands_.isActive[iDevice] = true;
        machine.commands_.velocity[iDevice] = 0.0;
        machine.commands_.position[iDevice] = motor->getPosition();
	}
}

StFault::~StFault() {
	ROS_INFO("Exiting StFault");
}

sc::result StFault::react( const EvExecute& )
{
	return forward_event();
}

sc::result StFault::react( const EvEmergencyStop& )
{
	return discard_event();
}

sc::result StFault::react( const EvStateInfo& )
{
	outermost_context().actualState_ = SM_FAULT;
	ROS_INFO("StFault");
	return discard_event();
}

//////////////////////////////////////////////////////////////////////////////
StDriveTop::StDriveTop( my_context ctx ) :
  my_base( ctx ), counter_(0)
{
	ROS_INFO("Entering StDriveTop");
}

StDriveTop::~StDriveTop() {
	ROS_INFO("Exiting StDriveTop");
}

sc::result StDriveTop::react( const EvExecute& )
{
	outermost_context_type & machine = outermost_context();

	if (counter_ > 50) {
		DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();
		for (int iDevice=0; iDevice < devices->getSize(); iDevice++) {
			DeviceELMOBaseMotor* motor =  (DeviceELMOBaseMotor*) devices->getDevice(iDevice);
			if (motor->getStatus()->isFault()) {
				return transit<StFault>();
			}
			if (motor->getStatus()->isDisabled()) {
				return transit<StStop>();
			}
		}
	} else {
		counter_++;
	}

	return forward_event();
}


//////////////////////////////////////////////////////////////////////////////
StDrive::StDrive( my_context ctx ) :
  my_base( ctx )
{
	ROS_INFO("Entering StDrive");
	outermost_context_type & machine = outermost_context();
	DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();
	for (int iDevice=0; iDevice < devices->getSize(); iDevice++) {
		DeviceELMOBaseMotor* motor =  (DeviceELMOBaseMotor*) devices->getDevice(iDevice);
		motor->setEnableMotor();
		motor->commandIsActive_ = true;
		motor->commandPosition_ = motor->getPosition();
	}
}

StDrive::~StDrive() {
	ROS_INFO("Exiting StDrive");
	outermost_context_type & machine = outermost_context();
	DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();
	for (int iDevice=0; iDevice < devices->getSize(); iDevice++) {
		DeviceELMOBaseMotor* motor =  (DeviceELMOBaseMotor*) devices->getDevice(iDevice);
		motor->setDisableMotor();
	}
}

sc::result StDrive::react( const EvExecute& )
{
	outermost_context_type & machine = outermost_context();

	DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();


	/* set commands of driving motors */
	for (int iDevice=0; iDevice < 6; iDevice++) {
		DeviceELMODrivingMotor* motor =  (DeviceELMODrivingMotor*) devices->getDevice(iDevice);
		if (machine.commands_.isActive[iDevice]) {
			motor->setProfileVelocity(machine.commands_.velocity[iDevice]);
		} else {
			motor->setProfileVelocity(0.0);
		}
	}

	/* set commands of steering motors */
	for (int iDevice=6; iDevice < 10; iDevice++) {
		DeviceELMOSteeringMotor* motor =  (DeviceELMOSteeringMotor*) devices->getDevice(iDevice);
		if (machine.commands_.isActive[iDevice]) {
			/* send command position */
            if (machine.commands_.velocity[iDevice] == 0.0) {
                // Set position without velocity profile
                motor->setProfilePosition(machine.commands_.position[iDevice]);
            } else {
                // Set position with velocity profile
#warning Position with velocity is not implemented
                motor->setProfilePosition(machine.commands_.position[iDevice]);
            }
		} else {
			/* ROS command is not active */
			if (motor->commandIsActive_) {
				/* the last command was active, i.e. store actual position */
				motor->commandPosition_ = motor->getPosition();
			}
			/* send actual position */
			motor->setProfilePosition(motor->commandPosition_);
		}
		motor->commandIsActive_ = machine.commands_.isActive[iDevice];
	}

	return forward_event();
}


sc::result StDrive::react( const EvStateInfo& )
{
	outermost_context().actualState_ = SM_DRIVE;
	ROS_INFO("StDrive");
	return discard_event();
}


//////////////////////////////////////////////////////////////////////////////
StDriveTestDrivingMotor::StDriveTestDrivingMotor( my_context ctx ) :
  my_base( ctx )
{
	ROS_INFO("Entering StDriveTestDrivingMotor");
	outermost_context_type & machine = outermost_context();
	DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();
	for (int iDevice=0; iDevice < devices->getSize(); iDevice++) {
		DeviceELMODrivingMotor* motor =  (DeviceELMODrivingMotor*) devices->getDevice(iDevice);
		motor->setEnableMotor();
	}

}

StDriveTestDrivingMotor::~StDriveTestDrivingMotor() {
	ROS_INFO("Exiting StDriveTestDrivingMotor");
	outermost_context_type & machine = outermost_context();
	DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();
	for (int iDevice=0; iDevice < devices->getSize(); iDevice++) {
		DeviceELMODrivingMotor* motor =  (DeviceELMODrivingMotor*) devices->getDevice(iDevice);
		motor->setDisableMotor();
	}
}

sc::result StDriveTestDrivingMotor::react( const EvExecute& )
{
	outermost_context_type & machine = outermost_context();

	/* initialize all devices */
	DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();
	for (int iDevice=0; iDevice < devices->getSize(); iDevice++) {
		DeviceELMODrivingMotor* motor =  (DeviceELMODrivingMotor*) devices->getDevice(iDevice);
		motor->setProfileVelocity(1.0);
	}

	return forward_event();
}


sc::result StDriveTestDrivingMotor::react( const EvStateInfo& )
{
	outermost_context().actualState_ = SM_DRIVE;
	ROS_INFO("StDriveTestDrivingMotor");
	return discard_event();
}

//////////////////////////////////////////////////////////////////////////////
StDriveTestSteeringMotor::StDriveTestSteeringMotor( my_context ctx ) :
  my_base( ctx )
{
	ROS_INFO("Entering StDriveTestSteeringMotor");
	outermost_context_type & machine = outermost_context();
	DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();
	for (int iDevice=0; iDevice < devices->getSize(); iDevice++) {
		DeviceELMOSteeringMotor* motor =  (DeviceELMOSteeringMotor*) devices->getDevice(iDevice);
		motor->setEnableMotor();
	}
}

StDriveTestSteeringMotor::~StDriveTestSteeringMotor() {
	ROS_INFO("Exiting StDriveTestSteeringMotor");
	outermost_context_type & machine = outermost_context();
	DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();
	for (int iDevice=0; iDevice < devices->getSize(); iDevice++) {
		DeviceELMOSteeringMotor* motor =  (DeviceELMOSteeringMotor*) devices->getDevice(iDevice);
		motor->setDisableMotor();
	}
}

sc::result StDriveTestSteeringMotor::react( const EvExecute& )
{
	outermost_context_type & machine = outermost_context();

	/* initialize all devices */
	DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();
	for (int iDevice=0; iDevice < devices->getSize(); iDevice++) {
		DeviceELMOSteeringMotor* motor =  (DeviceELMOSteeringMotor*) devices->getDevice(iDevice);
		motor->setProfilePosition(3.2);
	}

	return forward_event();
}


sc::result StDriveTestSteeringMotor::react( const EvStateInfo& )
{
	outermost_context().actualState_ = SM_DRIVE;
	ROS_INFO("StDriveTestSteeringMotor");
	return discard_event();
}

//////////////////////////////////////////////////////////////////////////////
StDriveTestAll::StDriveTestAll( my_context ctx ) :
  my_base( ctx )
{
	ROS_INFO("Entering StDriveTestSteeringMotor");
	outermost_context_type & machine = outermost_context();
	DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();
	for (int iDevice=0; iDevice < devices->getSize(); iDevice++) {
		DeviceELMOBaseMotor* motor =  (DeviceELMOBaseMotor*) devices->getDevice(iDevice);
		motor->setEnableMotor();
	}
}

StDriveTestAll::~StDriveTestAll() {
	ROS_INFO("Exiting StDriveTestSteeringMotor");
	outermost_context_type & machine = outermost_context();
	DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();
	for (int iDevice=0; iDevice < devices->getSize(); iDevice++) {
		DeviceELMOBaseMotor* motor =  (DeviceELMOBaseMotor*) devices->getDevice(iDevice);
		motor->setDisableMotor();
	}
}

sc::result StDriveTestAll::react( const EvExecute& )
{
	outermost_context_type & machine = outermost_context();
	DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();

	/* set commands of driving motors */
	for (int iDevice=0; iDevice < 6; iDevice++) {
		DeviceELMODrivingMotor* motor =  (DeviceELMODrivingMotor*) devices->getDevice(iDevice);
		motor->setProfileVelocity(1.0);
	}

	/* set commands of steering motors */
	for (int iDevice=6; iDevice < 10; iDevice++) {
		DeviceELMOSteeringMotor* motor =  (DeviceELMOSteeringMotor*) devices->getDevice(iDevice);
		motor->setProfilePosition(1.2*M_PI/2.0);

	}

	return forward_event();
}


sc::result StDriveTestAll::react( const EvStateInfo& )
{
	outermost_context().actualState_ = SM_DRIVE;
	ROS_INFO("StDriveTestAll");
	return discard_event();
}




