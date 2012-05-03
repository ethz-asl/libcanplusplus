/*!
 * @file 	DeviceELMOSteeringMotorVel.cpp
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, device
 *
 */

#include "DeviceELMOSteeringMotorVel.hpp"
#include <stdio.h>
#include <math.h>


DeviceELMOSteeringMotorVel::DeviceELMOSteeringMotorVel(int nodeId, DeviceELMOMotorParametersHDPC* deviceParams)
:DeviceELMOBaseMotor(nodeId,deviceParams)
{

}

DeviceELMOSteeringMotorVel::~DeviceELMOSteeringMotorVel()
{

}


void DeviceELMOSteeringMotorVel::addRxPDOs()
{

	/* add Velocity RxPDO */
	rxPDOVelocity_ = new RxPDOVelocity(nodeId_, deviceParams_->rxPDO1SMId_);
	bus_->getRxPDOManager()->addPDO(rxPDOVelocity_);

}

void DeviceELMOSteeringMotorVel::setProfilePosition(double jointPosition_rad, double jointVelocity_rad_s)
{
    const double Ke = -1.0;
    double command_pos = remainder(jointPosition_rad,M_PI);
    double position_error = remainder(command_pos - getPosition(), 2*M_PI);
    double command_vel = Ke * position_error;
    if (command_vel > jointVelocity_rad_s) {command_vel = jointVelocity_rad_s;}
    if (command_vel <-jointVelocity_rad_s) {command_vel =-jointVelocity_rad_s;}
    printf("%d:Pos control: C %.2f S %.2f O %.2f\n",nodeId_,jointPosition_rad,
	getPosition(), command_vel);
    setProfileVelocity(command_vel);
}

void DeviceELMOSteeringMotorVel::setProfileVelocity(double jointVelocity_rad_s)
{
	int jointVelocity_counts_s = jointVelocity_rad_s * deviceParams_->rad_s_Gear_to_counts_s_Motor;
	rxPDOVelocity_->setVelocity(jointVelocity_counts_s);
}

void DeviceELMOSteeringMotorVel::setMotorParameters()
{
	SDOManager* SDOManager = bus_->getSDOManager();
	setPositionLimits(deviceParams_->positionLimits);

	SDOManager->addSDO(new SDOSetOperationMode(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, deviceParams_->operationMode));
	SDOManager->addSDO(new SDOSetDS402ConfigurationObject(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x02)); //0x02
	SDOManager->addSDO(new SDOSetPPModeProfileVelocity(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, deviceParams_->profileVelocityInPPMode)); //Set profile velocity to 1000RPM

}


bool DeviceELMOSteeringMotorVel::initDevice()
{
	SDOManager* SDOManager = bus_->getSDOManager();

//	SDOManager->addSDO(new SDONMTResetCommunication(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
//	SDOManager->addSDO(new SDONMTResetNode(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));


//	printf("NMT: Enter Pre-Operational\n");
	SDOManager->addSDO(new SDONMTEnterPreOperational(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	SDOManager->addSDO(new SDOSetCOBIDSYNC(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x80));



	/* configure the PDOs on the motor controller */
	configTxPDOs();
	configRxPDOs();

	/* configure several motor parameters */
	setMotorParameters();
	initMotor();


//	printf("NMT: Start remote node\n");
	SDOManager->addSDO(new SDONMTStartRemoteNode(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));

	return true;

}


void DeviceELMOSteeringMotorVel::configRxPDOs()
{

	SDOManager* SDOManager = bus_->getSDOManager();

	/* deactivate all RxPDOs */
	SDOManager->addSDO(new SDORxPDO1SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));
	SDOManager->addSDO(new SDORxPDO2SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));
	SDOManager->addSDO(new SDORxPDO3SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));
	SDOManager->addSDO(new SDORxPDO4SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));

	configRxPDOProfileVelocity();
}


void DeviceELMOSteeringMotorVel::configRxPDOProfileVelocity()
{
	SDOManager* SDOManager = bus_->getSDOManager();

	/* Receive PDO 1 Parameter */
	///< Step 1: Configure COB-ID of the RxPDO 2
	SDOManager->addSDO(new SDORxPDO2ConfigureCOBID(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	///< Step 2: Set Transmission Type: SYNC 0x01
	SDOManager->addSDO(new SDORxPDO2SetTransmissionType(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x01)); // SYNC
	///< Step 3: Number of Mapped Application Objects
	SDOManager->addSDO(new SDORxPDO2SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));
	///< Step 4: Mapping Objects

	///< Mapping "Operation Mode"
	SDOManager->addSDO(new SDORxPDO2SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x01, 0x60600008));
	///< Mapping "Demand Velocity"
	SDOManager->addSDO(new SDORxPDO2SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x02, 0x60FF0020));
	///< Mapping "Controlword"
	SDOManager->addSDO(new SDORxPDO2SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x03, 0x60400010));

	///< Step 5: Number of Mapped Application Objects
	SDOManager->addSDO(new SDORxPDO2SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x03));

}

void DeviceELMOSteeringMotorVel::setEnableMotor()
{
	rxPDOVelocity_->enable();
}

void DeviceELMOSteeringMotorVel::setDisableMotor()
{
	rxPDOVelocity_->disable();
}



