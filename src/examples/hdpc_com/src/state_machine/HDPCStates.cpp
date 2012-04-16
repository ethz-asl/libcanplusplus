
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
//	return transit<StInitFault>();
	return discard_event();
}

sc::result StTop::react( const EvTerminateSM& )
{
	return terminate();
}


//////////////////////////////////////////////////////////////////////////////
StInit::StInit( my_context ctx ) :
  my_base( ctx )
{
	ROS_INFO("Entering StInit");
}

StInit::~StInit() {
	ROS_INFO("Exiting StInit");
}

sc::result StInit::react( const EvExecute& )
{
	outermost_context_type & machine = outermost_context();

	/* initialize all devices */
	DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();
	for (int iDevice=0; iDevice < devices->getSize(); iDevice++) {
		DeviceELMOBaseMotor* motor =  (DeviceELMOBaseMotor*) devices->getDevice(iDevice);
		motor->initDevice();
	}

	return discard_event();
}

sc::result StInit::react( const EvEmergencyStop& )
{
//	return transit<StInitFault>();
	return forward_event();
}

sc::result StInit::react( const EvStateInfo& )
{
	outermost_context().actualState_ = HDPCStateMachine::SM_INITIALIZING;
	ROS_INFO("StInit");
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
//	return transit<StInitFault>();
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
		motor->initDevice();
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

	/* initialize all devices */
	DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();
	for (int iDevice=0; iDevice < devices->getSize(); iDevice++) {
		DeviceELMOBaseMotor* motor =  (DeviceELMOBaseMotor*) devices->getDevice(iDevice);
		motor->initDevice();
	}

	return discard_event();
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
StFault::StFault( my_context ctx ) :
  my_base( ctx )
{
	ROS_INFO("Entering StFault");
}

StFault::~StFault() {
	ROS_INFO("Exiting StFault");
}

sc::result StFault::react( const EvExecute& )
{
	outermost_context_type & machine = outermost_context();

	/* initialize all devices */
	DeviceManager* devices = machine.busManager_->getBus(0)->getDeviceManager();
	for (int iDevice=0; iDevice < devices->getSize(); iDevice++) {
		DeviceELMOBaseMotor* motor =  (DeviceELMOBaseMotor*) devices->getDevice(iDevice);
		motor->initDevice();
	}

	return discard_event();
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
