/*!
 * @file 	DeviceELMOMotor.cpp
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, device
 *
 */

#include "DeviceELMOMotor.hpp"
#include <stdio.h>
#include <math.h>


DeviceELMOMotor::DeviceELMOMotor(int nodeId, DeviceELMOMotorParameters* deviceParams)
:Device(nodeId),deviceParams_(deviceParams)
{
	sdoStatusWord_ =  SDOReadStatusWord::SDOReadStatusWordPtr(new SDOReadStatusWord(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	sdoStatusWordDisabled_ = SDOReadStatusWord::SDOReadStatusWordPtr(new SDOReadStatusWord(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	sdoAnalogInputOne_ = SDOGetAnalogInputOne::SDOGetAnalogInputOnePtr(new SDOGetAnalogInputOne(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
}

DeviceELMOMotor::~DeviceELMOMotor()
{
	if (deviceParams_ != NULL){
		delete deviceParams_;
	}

}


DeviceELMOMotorParameters* DeviceELMOMotor::getDeviceParams()
{
	return deviceParams_;
}

void DeviceELMOMotor::addRxPDOs()
{
	/* add Velocity RxPDO */
	rxPDOVelocity_ = new RxPDOVelocity(nodeId_, deviceParams_->rxPDO1SMId_);
	bus_->getRxPDOManager()->addPDO(rxPDOVelocity_);

	/* add Position RxPDO */
	rxPDOPosition_ = new RxPDOPosition(nodeId_, deviceParams_->rxPDO1SMId_);
	//bus_->getRxPDOManager()->addPDO(rxPDOPosition_);

	rxPDOELMOBinaryInterpreterCmd_ = new RxPDOELMOBinaryInterpreterCmd(nodeId_, "an", 1, deviceParams_->rxPDO2SMId_);
	//bus_->getRxPDOManager()->addPDO(rxPDOELMOBinaryInterpreterCmd_);
}

void DeviceELMOMotor::addTxPDOs()
{
	/* add PositionVelocity TxPDO */
	txPDOPositionVelocity_ = new TxPDOPositionVelocity(nodeId_, deviceParams_->txPDO3SMId_);
	bus_->getTxPDOManager()->addPDO(txPDOPositionVelocity_);

	txPDOAnalogCurrent_ = new TxPDOAnalogCurrent(nodeId_, deviceParams_->txPDO4SMId_);
	bus_->getTxPDOManager()->addPDO(txPDOAnalogCurrent_);
}


void DeviceELMOMotor::setVelocity(double jointVelocity_rad_s)
{
	int jointVelocity_counts_s = jointVelocity_rad_s * deviceParams_->rad_s_Gear_to_counts_s_Motor;
	rxPDOVelocity_->setVelocity(jointVelocity_counts_s);
}
void DeviceELMOMotor::setPosition(double jointPosition_rad)
{

	int jointPosition_ticks = jointPosition_rad * deviceParams_->gearratio_motor * deviceParams_->RAD_TO_TICKS;
	rxPDOPosition_->setPosition(jointPosition_ticks);
	//jointPosition_ticks = 500000;
//	Working with elmo
	//	SDOManager* SDOManager = bus_->getSDOManager();
	//	SDOManager->addSDO(new SDOSetProfilePosition(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, jointPosition_ticks));
	//SDOManager->addSDO(new SDOControlWord(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x03F));


}

double DeviceELMOMotor::getPosition()
{
	return ((double) txPDOPositionVelocity_->getPosition()) / ( deviceParams_->gearratio_motor * deviceParams_->RAD_TO_TICKS);

}

double DeviceELMOMotor::getVelocity()
{

	return ((double)txPDOPositionVelocity_->getVelocity()) / ( deviceParams_->rad_s_Gear_to_counts_s_Motor);
}

double DeviceELMOMotor::getCurrent()
{
	return ((double) txPDOAnalogCurrent_->getCurrent()) * deviceParams_->continuous_current_limit / 1000.0;
}


double DeviceELMOMotor::getAnalog()
{
	return ((double) txPDOAnalogCurrent_->getAnalog())*0.00067139;
}


void DeviceELMOMotor::setPositionLimits(double * positionLimit_rad)
{


	double minPositionLimit, maxPositionLimit;

	if (positionLimit_rad[0] < positionLimit_rad[1]) {
		minPositionLimit = positionLimit_rad[0];
		maxPositionLimit = positionLimit_rad[1];
	} else {
		minPositionLimit = positionLimit_rad[1];
		maxPositionLimit = positionLimit_rad[0];
	}

	int minLimit_ticks =  (int) (minPositionLimit * deviceParams_->gearratio_motor * deviceParams_->RAD_TO_TICKS);
	int maxLimit_ticks =  (int) (maxPositionLimit * deviceParams_->gearratio_motor * deviceParams_->RAD_TO_TICKS);

	bus_->getSDOManager()->addSDO(new SDOSetMinPositionLimit(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, minLimit_ticks));
	bus_->getSDOManager()->addSDO(new SDOSetMaxPositionLimit(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, maxLimit_ticks));

}

void DeviceELMOMotor::setMotorParameters()
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


bool DeviceELMOMotor::initDevice()
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

void DeviceELMOMotor::configTxPDOs()
{
	SDOManager* SDOManager = bus_->getSDOManager();

	/* deactivate all TxPDOs */
	SDOManager->addSDO(new SDOTxPDO1SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));
	SDOManager->addSDO(new SDOTxPDO2SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));
	SDOManager->addSDO(new SDOTxPDO3SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));
	SDOManager->addSDO(new SDOTxPDO4SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));

	configTxPDOPositionVelocity();
	configTxPDOAnalogCurrent();
}

void DeviceELMOMotor::configRxPDOs()
{

	SDOManager* SDOManager = bus_->getSDOManager();

	/* deactivate all RxPDOs */
	SDOManager->addSDO(new SDORxPDO1SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));
	SDOManager->addSDO(new SDORxPDO2SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));
	SDOManager->addSDO(new SDORxPDO3SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));
	SDOManager->addSDO(new SDORxPDO4SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));


	configRxPDOProfileVelocity();
	//configRxPDOPosition();
}

void DeviceELMOMotor::configTxPDOPositionVelocity()
{
	SDOManager* SDOManager = bus_->getSDOManager();

	// Transmit PDO 3 Parameter
	///< configure COB-ID Transmit PDO 3
	SDOManager->addSDO(new SDOTxPDO3ConfigureCOBID(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	///< Set Transmission Type: SYNC 0x01
	SDOManager->addSDO(new SDOTxPDO3SetTransmissionType(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x01)); // SYNC
	///< Number of Mapped Application Objects
	SDOManager->addSDO(new SDOTxPDO3SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));
	///< Mapping "Position actual value"
	SDOManager->addSDO(new SDOTxPDO3SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x01, 0x60640020));
	///< Mapping "demand Velocity actual value"
	// velocity demand value 0x606B0020
	// velocity target value 0x60FF0020
	// working:
	//SDOManager->addSDO(new SDOTxPDO3SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x02, 0x606B0020));
	SDOManager->addSDO(new SDOTxPDO3SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x02, 0x60690020));
	///< Number of Mapped Application Objects
	SDOManager->addSDO(new SDOTxPDO3SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x02));

}

void DeviceELMOMotor::configTxPDOAnalogCurrent()
{
//	SDOManager* SDOManager = bus_->getSDOManager();
//
//	// Transmit PDO 4 Parameter
//	///< configure COB-ID Transmit PDO 4
//	SDOManager->addSDO(new SDOTxPDO1ConfigureCOBID(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
//	///< Set Transmission Type: SYNC 0x01
//	SDOManager->addSDO(new SDOTxPDO1SetTransmissionType(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x01)); // SYNC
//	///< Number of Mapped Application Objects
//	SDOManager->addSDO(new SDOTxPDO1SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));
//	///< Mapping "Velocity actual value"
//	SDOManager->addSDO(new SDOTxPDO1SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x01, 0x60640020));
//	///< Mapping "actual current value"
//	SDOManager->addSDO(new SDOTxPDO1SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x02, 0x60780010));
//	///< Number of Mapped Application Objects
//	SDOManager->addSDO(new SDOTxPDO1SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x02));

// analog and current
	SDOManager* SDOManager = bus_->getSDOManager();

	// Transmit PDO 4 Parameter
	///< configure COB-ID Transmit PDO 4
	SDOManager->addSDO(new SDOTxPDO4ConfigureCOBID(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	///< Set Transmission Type: SYNC 0x01
	SDOManager->addSDO(new SDOTxPDO4SetTransmissionType(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x01)); // SYNC
	///< Number of Mapped Application Objects
	SDOManager->addSDO(new SDOTxPDO4SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));
	///< Mapping "Analog value"
	SDOManager->addSDO(new SDOTxPDO4SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x01, 0x22050110));
	///< Mapping "Digital value"
	/*	SDOManager->addSDO(new SDOTxPDO4SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x02, 0x22000020));*/

	///< Mapping "actual current value - works!"
	SDOManager->addSDO(new SDOTxPDO4SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x02, 0x60780010));


	///< Number of Mapped Application Objects
	SDOManager->addSDO(new SDOTxPDO4SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x02));


}

void DeviceELMOMotor::configRxPDOProfileVelocity()
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

void DeviceELMOMotor::configRxPDOPosition()
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

void DeviceELMOMotor::initMotor()
{
	SDOManager* SDOManager = bus_->getSDOManager();
	SDOManager->addSDO(new SDOFaultReset(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	SDOManager->addSDO(new SDOShutdown(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	SDOManager->addSDO(new SDOSwitchOn(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	SDOManager->addSDO(new SDOEnableOperation(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
}


void DeviceELMOMotor::setEnableMotor()
{
	SDOManager* SDOManager = bus_->getSDOManager();
	SDOManager->addSDO(new SDOFaultReset(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	SDOManager->addSDO(new SDOShutdown(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	SDOManager->addSDO(new SDOSwitchOn(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	SDOManager->addSDO(new SDOEnableOperation(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
}

void DeviceELMOMotor::setDisableMotor()
{
	SDOManager* SDOManager = bus_->getSDOManager();
	SDOManager->addSDO(new SDODisableOperation(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
}


bool DeviceELMOMotor::getIsMotorEnabled(bool &flag)
{
	SDOManager* SDOManager = bus_->getSDOManager();


	if (!sdoStatusWord_->hasTimeOut()) {
		if (!sdoStatusWord_->getIsReceived()) {
			if (!sdoStatusWord_->getIsWaiting()) {
				if (!sdoStatusWord_->getIsQueuing()) {
					SDOManager->addSDO((SDOMsgPtr)sdoStatusWord_);
				}
			}
		} else {
			sdoStatusWord_->isEnabled(flag);
			sdoStatusWord_.reset();
			sdoStatusWord_ =  SDOReadStatusWord::SDOReadStatusWordPtr(new SDOReadStatusWord(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
			return true;
		}
	}

	return false;

}

bool DeviceELMOMotor::getIsMotorDisabled(bool &flag)
{
	SDOManager* SDOManager = bus_->getSDOManager();


	if (!sdoStatusWordDisabled_->hasTimeOut()) {
		if (!sdoStatusWordDisabled_->getIsReceived()) {
			if (!sdoStatusWordDisabled_->getIsWaiting()) {
				if (!sdoStatusWordDisabled_->getIsQueuing()) {
					SDOManager->addSDO((SDOMsgPtr)sdoStatusWordDisabled_);
				}
			}
		} else {
			sdoStatusWordDisabled_->isDisabled(flag);
			sdoStatusWordDisabled_.reset();
			sdoStatusWordDisabled_ =  SDOReadStatusWord::SDOReadStatusWordPtr(new SDOReadStatusWord(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
			return true;
		}
	}

	return false;

}

bool DeviceELMOMotor::getAnalogInputOne(double& value)
{
	SDOManager* SDOManager = bus_->getSDOManager();


	if (!sdoAnalogInputOne_->hasTimeOut()) {
		if (!sdoAnalogInputOne_->getIsReceived()) {
			if (!sdoAnalogInputOne_->getIsWaiting()) {
				if (!sdoAnalogInputOne_->getIsQueuing()) {
					SDOManager->addSDO((SDOMsgPtr)sdoAnalogInputOne_);
				}
			}
		} else {
			int val;
			sdoAnalogInputOne_->getAnalog(val);
			value = (double) val;
			sdoAnalogInputOne_.reset();
			sdoAnalogInputOne_ =  SDOGetAnalogInputOne::SDOGetAnalogInputOnePtr(new SDOGetAnalogInputOne(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
			return true;
		}
	}

	return false;
}

