/*!
 * @file 	HDPCStates.cpp
 * @brief	States of the state machine for HDPC
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
	outermost_context_type & machine = outermost_context();
	machine.isInStFault_ = true;
	return transit<StStopping>();
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
}

StInit::~StInit() {
	ROS_INFO("Exiting StInit");
}

sc::result StInit::react( const EvExecute& )
{
	outermost_context_type & machine = outermost_context();

	/* The previous device should have some time to initialize
	 * counts ~= 2x no. of SDOs to init
	 */
	if (waitForDeviceCount_++ > 20) {
		waitForDeviceCount_ = 0;

		ROS_INFO("Init device %d", iDevice_);
		/* initialize device */
		DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();
		DeviceELMOBaseMotor* motor =  (DeviceELMOBaseMotor*) devices->getDevice(iDevice_);
		motor->initDevice();

		iDevice_++;

		/* check if all devices are initialized */
		if (iDevice_ >= devices->getSize()) {
			return transit<StHoming>();
		}
	}

	return forward_event();
}

sc::result StInit::react( const EvEmergencyStop& )
{
	return forward_event();
}

sc::result StInit::react( const EvStateInfo& )
{
	outermost_context().actualState_ = HDPCStateMachine::SM_INITIALIZING;
	ROS_INFO("StInit");
	return discard_event();
}
//////////////////////////////////////////////////////////////////////////////
StHoming::StHoming( my_context ctx ) :
  my_base( ctx )
{
	ROS_INFO("Entering StHoming");
}

StHoming::~StHoming() {
	ROS_INFO("Exiting StHoming");
}

sc::result StHoming::react( const EvExecute& )
{
	outermost_context_type & machine = outermost_context();

	/* The previous device should have some time to initialize
	 * counts ~= 2x no. of SDOs to init
	 */
	if (waitForDeviceCount_++ > 200) {
		waitForDeviceCount_ = 0;

		/* initialize device */
		DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();
		DeviceELMOBaseMotor* motor =  (DeviceELMOBaseMotor*) devices->getDevice(iDevice_);


		iDevice_++;

		/* check if all devices are initialized */
		if (iDevice_ >= devices->getSize()) {
			return transit<StStop>();
		}
	}

	return forward_event();
}

sc::result StHoming::react( const EvEmergencyStop& )
{
	return forward_event();
}

sc::result StHoming::react( const EvStateInfo& )
{
	outermost_context().actualState_ = HDPCStateMachine::SM_INITIALIZING;
	ROS_INFO("StHoming");
	return discard_event();
}

//////////////////////////////////////////////////////////////////////////////
StStopping::StStopping( my_context ctx ) :
  my_base( ctx ),sv_disabling_counter_(0), iDevice_(0)
{

	ROS_INFO("Entering StStopping");

	outermost_context_type & machine = outermost_context();

	const int nDevices = machine.busManager_->getBus(0)->getDeviceManager()->getSize();
	isDisabling_ = new bool[nDevices];
	isReallyDisabled_  = new bool[nDevices];

	for(int iDevice=0; iDevice<nDevices; iDevice++) {
		isDisabling_[iDevice] = false;
		isReallyDisabled_[iDevice] = false;
	}
	machine.busManager_->getBus(0)->getRxPDOManager()->setSending(false);
}

StStopping::~StStopping() {

	ROS_INFO("Exiting StStopping");

	delete[] isDisabling_;
	delete[] isReallyDisabled_;
	outermost_context_type & machine = outermost_context();
	machine.busManager_->getBus(0)->getRxPDOManager()->setSending(true);
}

sc::result StStopping::react( const EvExecute& )
{
	outermost_context_type & machine = outermost_context();
	const int nDevices = machine.busManager_->getBus(0)->getDeviceManager()->getSize();

	DeviceELMOBaseMotor* motor = (DeviceELMOBaseMotor*) machine.busManager_->getBus(0)->getDeviceManager()->getDevice(iDevice_);
	bool isMotorDisabled;
	if (motor->getIsMotorDisabled(isMotorDisabled)) {
		/* data is available whether the motor is enabled or not */
		if (!isMotorDisabled) {
			/* motor is enabled */
			if (!isDisabling_[iDevice_]) {
				/* motor should be enabled */
				motor->setDisableMotor();
				isDisabling_[iDevice_] = true;
			}

		} else {
			/* motor is disabled */
			isReallyDisabled_[iDevice_] = true;
			iDevice_++;
			sv_disabling_counter_=0;
		}
	}



	sv_disabling_counter_++;
	if (sv_disabling_counter_>100*2) {
		sv_disabling_counter_=0;
		iDevice_++;
	}


	if (iDevice_ >= nDevices) {
		bool allDevicesAreDisabled = true;
		for(int iDevice=0; iDevice<nDevices; iDevice++) {
			if (!isReallyDisabled_[iDevice]) {
				allDevicesAreDisabled = false;
				break;
			}
		}

		if (allDevicesAreDisabled) {
			if (machine.isInStFault_) {
				return transit< StFault >();
			}
			return transit< StStop >();
		} else {
			for(int iDevice=0; iDevice<nDevices; iDevice++) {
				if (!isReallyDisabled_[iDevice]) {
					ROS_INFO("\e[0;31mMotor %d is not disabled!\n\e[0m", iDevice);
				}
			}
			return transit< StFault >();
		}

	}
	return  forward_event();
}

sc::result StStopping::react( const EvEmergencyStop& )
{
	return forward_event();
}

sc::result StStopping::react( const EvStateInfo& )
{
	outermost_context().actualState_ = HDPCStateMachine::SM_STOPPING;
	ROS_INFO("StStopping");
	return discard_event();
}

//////////////////////////////////////////////////////////////////////////////
StStop::StStop( my_context ctx ) :
  my_base( ctx )
{
	ROS_INFO("Entering StStop");
}

StStop::~StStop() {
	ROS_INFO("Exiting StStop");
}

sc::result StStop::react( const EvExecute& )
{
	outermost_context_type & machine = outermost_context();

	/* initialize all devices */
	DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();
	for (int iDevice=0; iDevice < devices->getSize(); iDevice++) {
		DeviceELMOBaseMotor* motor =  (DeviceELMOBaseMotor*) devices->getDevice(iDevice);

	}

	return discard_event();
}

sc::result StStop::react( const EvEmergencyStop& )
{
	return forward_event();
}

sc::result StStop::react( const EvStateInfo& )
{
	outermost_context().actualState_ = HDPCStateMachine::SM_STOP;
	ROS_INFO("StStop");
	return discard_event();
}


//////////////////////////////////////////////////////////////////////////////
StStarting::StStarting( my_context ctx ): my_base( ctx ),
		sv_enabling_counter_(0),iDevice_(0)
{
	ROS_INFO("Entering StStarting");
	outermost_context_type & machine = outermost_context();
	const int nDevices = machine.busManager_->getBus(0)->getDeviceManager()->getSize();
	isEnabling_ = new bool[nDevices];
	isReallyEnabled_  = new bool[nDevices];

	for(int iDevice=0; iDevice<nDevices; iDevice++) {
		isEnabling_[iDevice] = false;
		isReallyEnabled_[iDevice] = false;
	}
	machine.busManager_->getBus(0)->getRxPDOManager()->setSending(false);
}

StStarting::~StStarting()
{
	ROS_INFO("Exiting StStarting");

	delete[] isEnabling_;
	delete[] isReallyEnabled_;
	outermost_context_type & machine = outermost_context();
	machine.busManager_->getBus(0)->getRxPDOManager()->setSending(true);
}

sc::result StStarting::react( const EvExecute & )
{

	outermost_context_type & machine = outermost_context();
	const int nDevices = machine.busManager_->getBus(0)->getDeviceManager()->getSize();

	DeviceELMOBaseMotor* motor = (DeviceELMOBaseMotor*) machine.busManager_->getBus(0)->getDeviceManager()->getDevice(iDevice_);
	bool isMotorEnabled;
	if (motor->getIsMotorEnabled(isMotorEnabled)) {
		/* data is available whether the motor is enabled or not */
		if (!isMotorEnabled) {
			/* motor is not enabled */
			if (!isEnabling_[iDevice_]) {
				/* motor should be enabled */
				motor->setEnableMotor();
				isEnabling_[iDevice_] = true;
			}
		} else {
			/* motor is enabled */
			isReallyEnabled_[iDevice_] = true;
		}
	}


	sv_enabling_counter_++;
	if (sv_enabling_counter_>100*2) {
		sv_enabling_counter_=0;
		iDevice_++;
	}
	if (iDevice_ >= nDevices) {
		bool allDevicesAreEnabled = true;
		for(int iDevice=0; iDevice<nDevices; iDevice++) {
			if (!isReallyEnabled_[iDevice]) {
				allDevicesAreEnabled = false;
				break;
			}
		}

		if (allDevicesAreEnabled) {
			return transit< StDrive >();
		} else if (sv_enabling_counter_ > 100*2) {
			for(int iDevice=0; iDevice<nDevices; iDevice++) {
				if (!isReallyEnabled_[iDevice]) {
					ROS_INFO("\e[0;31mMotor %d cannot be enabled!\n\e[0m", iDevice);
				}
			}
			return transit< StFault >();
		}
	}
	return  forward_event();

}

sc::result StStarting::react( const EvEmergencyStop& )
{
	return forward_event();
}

sc::result StStarting::react( const EvStateInfo& )
{
	outermost_context().actualState_ = HDPCStateMachine::SM_STARTING;
	ROS_INFO("StStarting");
	return discard_event();
}

//////////////////////////////////////////////////////////////////////////////
StDrive::StDrive( my_context ctx ) :
  my_base( ctx )
{
	ROS_INFO("Entering StDrive");
}

StDrive::~StDrive() {
	ROS_INFO("Exiting StDrive");
}

sc::result StDrive::react( const EvExecute& )
{
	outermost_context_type & machine = outermost_context();

	DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();

	/* set commands of driving motors */
	for (int iDevice=0; iDevice < 6; iDevice++) {
		DeviceELMODrivingMotor* motor =  (DeviceELMODrivingMotor*) devices->getDevice(iDevice);
		if (machine.commands_.isActive[iDevice]) {
			motor->setProfileVelocity(machine.commands_.command[iDevice]);
		}
	}

	/* set commands of steering motors */
	for (int iDevice=6; iDevice < 10; iDevice++) {
		DeviceELMOSteeringMotor* motor =  (DeviceELMOSteeringMotor*) devices->getDevice(iDevice);
		if (machine.commands_.isActive[iDevice]) {
			motor->setProfilePosition(machine.commands_.command[iDevice]);
		}
	}

	return forward_event();
}

sc::result StDrive::react( const EvEmergencyStop& )
{
	return forward_event();
}

sc::result StDrive::react( const EvStateInfo& )
{
	outermost_context().actualState_ = HDPCStateMachine::SM_DRIVE;
	ROS_INFO("StDrive");
	return discard_event();
}


//////////////////////////////////////////////////////////////////////////////
StDriveTestDrivingMotor::StDriveTestDrivingMotor( my_context ctx ) :
  my_base( ctx )
{
	ROS_INFO("Entering StDriveTestDrivingMotor");
}

StDriveTestDrivingMotor::~StDriveTestDrivingMotor() {
	ROS_INFO("Exiting StDriveTestDrivingMotor");
}

sc::result StDriveTestDrivingMotor::react( const EvExecute& )
{
	outermost_context_type & machine = outermost_context();

	/* initialize all devices */
	DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();
	for (int iDevice=0; iDevice < devices->getSize(); iDevice++) {
		DeviceELMOBaseMotor* motor =  (DeviceELMOBaseMotor*) devices->getDevice(iDevice);

	}

	return discard_event();
}

sc::result StDriveTestDrivingMotor::react( const EvEmergencyStop& )
{
	return forward_event();
}

sc::result StDriveTestDrivingMotor::react( const EvStateInfo& )
{
	outermost_context().actualState_ = HDPCStateMachine::SM_DRIVE;
	ROS_INFO("StDriveTestDrivingMotor");
	return discard_event();
}

//////////////////////////////////////////////////////////////////////////////
StDriveTestSteeringMotor::StDriveTestSteeringMotor( my_context ctx ) :
  my_base( ctx )
{
	ROS_INFO("Entering StDriveTestSteeringMotor");
}

StDriveTestSteeringMotor::~StDriveTestSteeringMotor() {
	ROS_INFO("Exiting StDriveTestSteeringMotor");
}

sc::result StDriveTestSteeringMotor::react( const EvExecute& )
{
	outermost_context_type & machine = outermost_context();

	/* initialize all devices */
	DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();
	for (int iDevice=0; iDevice < devices->getSize(); iDevice++) {
		DeviceELMOBaseMotor* motor =  (DeviceELMOBaseMotor*) devices->getDevice(iDevice);

	}

	return discard_event();
}

sc::result StDriveTestSteeringMotor::react( const EvEmergencyStop& )
{
	return forward_event();
}

sc::result StDriveTestSteeringMotor::react( const EvStateInfo& )
{
	outermost_context().actualState_ = HDPCStateMachine::SM_DRIVE;
	ROS_INFO("StDriveTestSteeringMotor");
	return discard_event();
}


//////////////////////////////////////////////////////////////////////////////
StFault::StFault( my_context ctx ) :
  my_base( ctx )
{
	ROS_INFO("Entering StFault");
}

StFault::~StFault() {
	ROS_INFO("Exiting StFault");
	outermost_context_type & machine = outermost_context();
	machine.isInStFault_ = false;
}

sc::result StFault::react( const EvExecute& )
{
	outermost_context_type & machine = outermost_context();

//	/* initialize all devices */
//	DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();
//	for (int iDevice=0; iDevice < devices->getSize(); iDevice++) {
//		DeviceELMOBaseMotor* motor =  (DeviceELMOBaseMotor*) devices->getDevice(iDevice);
//		motor->initDevice();
//	}

	return forward_event();
}

sc::result StFault::react( const EvEmergencyStop& )
{
	return forward_event();
}

sc::result StFault::react( const EvStateInfo& )
{
	outermost_context().actualState_ = HDPCStateMachine::SM_FAULT;
	ROS_INFO("StFault");
	return discard_event();
}
