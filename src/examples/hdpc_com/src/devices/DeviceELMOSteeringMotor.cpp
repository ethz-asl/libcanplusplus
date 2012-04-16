/*!
 * @file 	DeviceELMOSteeringMotor.cpp
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, device
 *
 */

#include "DeviceELMOSteeringMotor.hpp"
#include <stdio.h>
#include <math.h>


DeviceELMOSteeringMotor::DeviceELMOSteeringMotor(int nodeId, DeviceELMOMotorParameters* deviceParams)
:DeviceELMOBaseMotor(nodeId,deviceParams)
{

}

DeviceELMOSteeringMotor::~DeviceELMOSteeringMotor()
{

}


void DeviceELMOSteeringMotor::addRxPDOs()
{

	/* add Position RxPDO */
	rxPDOPosition_ = new RxPDOPosition(nodeId_, deviceParams_->rxPDOSMId_);
	bus_->getRxPDOManager()->addPDO(rxPDOPosition_);

}


void DeviceELMOSteeringMotor::setProfilePosition(double jointPosition_rad)
{

	int jointPosition_ticks = jointPosition_rad * deviceParams_->gearratio_motor * deviceParams_->RAD_TO_TICKS;
	rxPDOPosition_->setPosition(jointPosition_ticks);
	//jointPosition_ticks = 500000;
//	Working with elmo
	//	SDOManager* SDOManager = bus_->getSDOManager();
	//	SDOManager->addSDO(new SDOSetProfilePosition(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, jointPosition_ticks));
	//SDOManager->addSDO(new SDOControlWord(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x03F));


}

void DeviceELMOSteeringMotor::setMotorParameters()
{

	SDOManager* SDOManager = bus_->getSDOManager();


	setPositionLimits(deviceParams_->positionLimits);

	//SDOManager->addSDO(new SDOSetDS402ConfigurationObject(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x0002));



	SDOManager->addSDO(new SDOSetOperationMode(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, deviceParams_->operationMode));
	//SDOManager->addSDO(new SDOSetSensorSelectionCode(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x0000));


	SDOManager->addSDO(new SDOSetDS402ConfigurationObject(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x02)); //0x02
	SDOManager->addSDO(new SDOSetPPModeProfileVelocity(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 40000)); //Set profile velocity to 1000RPM
	//SDOManager->addSDO(new SDORestoreAllDefaultParameters(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	//SDOManager->addSDO(new SDOSetMotorType(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, deviceParams_->motor_type));
	//SDOManager->addSDO(new SDOSetVelocityIGain(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, deviceParams_->velocity_I_Gain));

	/*
	SDOManager->addSDO(new SDOSetEncoderPulseNumber(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, deviceParams_->encoder_pulse_number));
	SDOManager->addSDO(new SDOSetPositionSensorType(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, deviceParams_->encoder_type));
	SDOManager->addSDO(new SDOSetPositionSensorPolarity(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, deviceParams_->encoder_polarity, deviceParams_->hall_polarity));
	SDOManager->addSDO(new SDOSetMotorType(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, deviceParams_->motor_type));
	SDOManager->addSDO(new SDOSetPolePairNumber(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, deviceParams_->pole_pair_number));
	SDOManager->addSDO(new SDOSetThermalTimeConstantWinding(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, deviceParams_->thermal_time_constant_winding));

	SDOManager->addSDO(new SDOSetContinuousCurrentLimit(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, (int)(deviceParams_->continuous_current_limit*1000.0)));
	SDOManager->addSDO(new SDOSetOutputCurrentLimit(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, (int)(deviceParams_->continuous_current_limit*1000.0)));


	SDOManager->addSDO(new SDOSetVelocityPGain(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, deviceParams_->velocity_P_Gain));
	SDOManager->addSDO(new SDOSetVelocityIGain(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, deviceParams_->velocity_I_Gain));
	SDOManager->addSDO(new SDOSetVelocityVelFFGain(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, deviceParams_->velocity_VFF_Gain));
	SDOManager->addSDO(new SDOSetVelocityAccFFGain(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, deviceParams_->velocity_AFF_Gain));

	SDOManager->addSDO(new SDOSetCurrentPGain(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, deviceParams_->current_P_Gain));
	SDOManager->addSDO(new SDOSetCurrentIGain(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, deviceParams_->current_I_Gain));

//	SDOManager->addSDO(new SDOSetMaxProfileVelocity(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_,  (int)(deviceParams_->max_profile_velocity *  deviceParams_->rad_s_Gear_to_rpm_Motor) ));
//	SDOManager->addSDO(new SDOSetProfileAcceleration(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_,  (int)(deviceParams_->profile_acceleration *  deviceParams_->rad_s_Gear_to_rpm_Motor) ));
//	SDOManager->addSDO(new SDOSetProfileDeceleration(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_,  (int)(deviceParams_->profile_decceleration *  deviceParams_->rad_s_Gear_to_rpm_Motor) ));

	SDOManager->addSDO(new SDOSetMaxFollowingError(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, (int)(deviceParams_->max_following_error * deviceParams_->gearratio_motor * deviceParams_->RAD_TO_TICKS) ));

	SDOManager->addSDO(new SDOSetGuardTime(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0)); //Set a guard time of x ms with a factor , if set_guard_time(0): Guarding disabled
	SDOManager->addSDO(new SDOSetLifeTimeFactor(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 1));




	SDOManager->addSDO(new SDOSetOperationMode(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, deviceParams_->operationMode));

	// 1=Fault signal only instead of Quickstop
	SDOManager->addSDO(new SDOSetAbortConnectionOptionCode(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x01));
*/
}


bool DeviceELMOSteeringMotor::initDevice()
{
	SDOManager* SDOManager = bus_->getSDOManager();

//	SDOManager->addSDO(new SDONMTResetCommunication(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
//	SDOManager->addSDO(new SDONMTResetNode(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));

	printf("NMT: Enter Pre-Operational\n");
	SDOManager->addSDO(new SDONMTEnterPreOperational(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	SDOManager->addSDO(new SDOSetCOBIDSYNC(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x80));


	/* configure the PDOs on the motor controller */
	configTxPDOs();
	configRxPDOs();

	/* configure several motor parameters */
	setMotorParameters();
	//SDOManager->addSDO(new SDOSaveAllParameters(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));

	initMotor();
	SDOManager->addSDO(new SDOSetOperationMode(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, deviceParams_->operationMode));

	printf("NMT: Start remote node\n");
	SDOManager->addSDO(new SDONMTStartRemoteNode(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));

	//SDOManager->addSDO(new SDOControlWord(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x03F));

	return true;

}


void DeviceELMOSteeringMotor::configRxPDOs()
{

	SDOManager* SDOManager = bus_->getSDOManager();

	/* deactivate all RxPDOs */
	SDOManager->addSDO(new SDORxPDO1SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));
	SDOManager->addSDO(new SDORxPDO2SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));
	SDOManager->addSDO(new SDORxPDO3SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));
	SDOManager->addSDO(new SDORxPDO4SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));


	configRxPDOProfilePosition();
}


void DeviceELMOSteeringMotor::configRxPDOProfilePosition()
{
	SDOManager* SDOManager = bus_->getSDOManager();

	/* Receive PDO 1 Parameter */
	///< Step 1: Configure COB-ID of the RxPDO 3
	SDOManager->addSDO(new SDORxPDO3ConfigureCOBID(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	///< Step 2: Set Transmission Type: SYNC 0x01
	SDOManager->addSDO(new SDORxPDO3SetTransmissionType(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00)); // SYNC
	///< Step 3: Number of Mapped Application Objects
	SDOManager->addSDO(new SDORxPDO3SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));
	///< Step 4: Mapping Objects


	///< Mapping "Operation Mode"
	//SDOManager->addSDO(new SDORxPDO3SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x01, 0x60600008));
	///< Mapping "Target position"
	SDOManager->addSDO(new SDORxPDO3SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x01, 0x607A0020));
	///< Mapping "Controlword"
	SDOManager->addSDO(new SDORxPDO3SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x02, 0x60400010));
	///< Mapping "Velocity"
	//SDOManager->addSDO(new SDORxPDO3SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x04, 0x607F0020));



	///< Step 5: Number of Mapped Application Objects
	SDOManager->addSDO(new SDORxPDO3SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x02));
}


