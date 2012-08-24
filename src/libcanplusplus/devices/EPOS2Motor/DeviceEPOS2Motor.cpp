/*!
 * @file 	DeviceEPOS2Motor.cpp
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, device
 *
 */

#include "DeviceEPOS2Motor.hpp"
#include <stdio.h>
#include <math.h>


DeviceEPOS2Motor::DeviceEPOS2Motor(int nodeId, DeviceEPOS2MotorParameters* deviceParams)
:Device(nodeId),deviceParams_(deviceParams)
{
    enabled_ = false;
    operation_mode_ = 0; // Undefined
	sdoStatusWord_ =  SDOReadStatusWord::SDOReadStatusWordPtr(new SDOReadStatusWord(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	sdoStatusWordDisabled_ = SDOReadStatusWord::SDOReadStatusWordPtr(new SDOReadStatusWord(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	sdoAnalogInputOne_ = SDOGetAnalogInputOne::SDOGetAnalogInputOnePtr(new SDOGetAnalogInputOne(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	sdoAnalogInputTwo_ = SDOGetAnalogInputTwo::SDOGetAnalogInputTwoPtr(new SDOGetAnalogInputTwo(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
}

DeviceEPOS2Motor::~DeviceEPOS2Motor()
{
	if (deviceParams_ != NULL){
		delete deviceParams_;
	}
    // No need to delete, the bus manager will take care of that
    // delete rxPDOVelocity_;
    // delete rxPDOPosition_;
    // delete txPDOPositionVelocity_;
    // delete txPDOAnalogCurrent_;
}


DeviceEPOS2MotorParameters* DeviceEPOS2Motor::getDeviceParams()
{
	return deviceParams_;
}

void DeviceEPOS2Motor::addRxPDOs()
{
    /* add Velocity RxPDO */
    rxPDOVelocity_ = new RxPDOVelocity(1,nodeId_, deviceParams_->rxPDO1SMId_);
    bus_->getRxPDOManager()->addPDO(rxPDOVelocity_);

    /* add Position RxPDO, not sure if it can share the same SMId */
    rxPDOPosition_ = new RxPDOPosition(2,nodeId_, deviceParams_->rxPDO2SMId_);
    bus_->getRxPDOManager()->addPDO(rxPDOPosition_);

    /* add Position Limit RxPDO */
    //	rxPDOPositionLimit_ = new RxPDOPositionLimit(nodeId_, deviceParams_->rxPDO2SMId_);
    //	bus_->getRxPDOManager()->addPDO(rxPDOPositionLimit_);
}

void DeviceEPOS2Motor::addTxPDOs()
{
	/* add PositionVelocity TxPDO */
	txPDOPositionVelocity_ = new TxPDOPositionVelocity(1,nodeId_, deviceParams_->txPDO1SMId_);
	bus_->getTxPDOManager()->addPDO(txPDOPositionVelocity_);

	txPDOAnalogCurrent_ = new TxPDOAnalogCurrent(2,nodeId_, deviceParams_->txPDO2SMId_);
	bus_->getTxPDOManager()->addPDO(txPDOAnalogCurrent_);
}


void DeviceEPOS2Motor::setVelocity(double jointVelocity_rad_s)
{
    switch (operation_mode_) {
        case OPERATION_MODE_VELOCITY:
            {
                int motorVelocity_rpm =  (int) (jointVelocity_rad_s * deviceParams_->rad_s_Gear_to_rpm_Motor);
                rxPDOVelocity_->setVelocity(motorVelocity_rpm);
            } 
            break;
        default:
            printf("Ignoring non velocity command for device %04x\n",nodeId_);
            // ignore
            break;
    }
}

void DeviceEPOS2Motor::setPosition(double jointPosition_rad)
{
    switch (operation_mode_) {
        case OPERATION_MODE_POSITION:
            {
                int jointPosition_ticks = jointPosition_rad * deviceParams_->gearratio_motor * deviceParams_->RAD_TO_TICKS;
                // printf("Device %04x moving to %d\n",nodeId_,jointPosition_ticks);
                rxPDOPosition_->setPosition(jointPosition_ticks);
            } 
            break;
        default:
            printf("Ignoring non position command for device %04x\n",nodeId_);
            // ignore
            break;
    }
}

//void DeviceEPOS2Motor::setPositionLimitsPerPDO(double * positionLimit_rad)
//{
//	double minPositionLimit, maxPositionLimit;
//
//	if (positionLimit_rad[0] < positionLimit_rad[1]) {
//		minPositionLimit = positionLimit_rad[0];
//		maxPositionLimit = positionLimit_rad[1];
//	} else {
//		minPositionLimit = positionLimit_rad[1];
//		maxPositionLimit = positionLimit_rad[0];
//	}
//
//	int minLimit_ticks =  (int) (minPositionLimit * deviceParams_->gearratio_motor * deviceParams_->RAD_TO_TICKS);
//	int maxLimit_ticks =  (int) (maxPositionLimit * deviceParams_->gearratio_motor * deviceParams_->RAD_TO_TICKS);
//
//	rxPDOPositionLimit_->setPositionLimit(minLimit_ticks, maxLimit_ticks);
//}


double DeviceEPOS2Motor::getPosition()
{
	return ((double) txPDOPositionVelocity_->getPosition()) / ( deviceParams_->gearratio_motor * deviceParams_->RAD_TO_TICKS);
}

double DeviceEPOS2Motor::getVelocity()
{
	return ((double)txPDOPositionVelocity_->getVelocity()) / ( deviceParams_->rad_s_Gear_to_rpm_Motor);
}

double DeviceEPOS2Motor::getAnalog()
{
    return txPDOAnalogCurrent_->getAnalog();
}

double DeviceEPOS2Motor::getCurrent()
{
    return txPDOAnalogCurrent_->getCurrent();
}

unsigned int DeviceEPOS2Motor::getStatusWord() 
{
    return txPDOAnalogCurrent_->getStatusWord();
}


void DeviceEPOS2Motor::setPositionLimits(double * positionLimit_rad)
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

void DeviceEPOS2Motor::setMotorParameters()
{

	SDOManager* SDOManager = bus_->getSDOManager();

	SDOManager->addSDO(new SDOSetEncoderPulseNumber(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, deviceParams_->encoder_pulse_number));
	SDOManager->addSDO(new SDOSetPositionSensorType(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, deviceParams_->encoder_type));
	SDOManager->addSDO(new SDOSetPositionSensorPolarity(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, deviceParams_->encoder_polarity, deviceParams_->hall_polarity));
	SDOManager->addSDO(new SDOSetMotorType(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, deviceParams_->motor_type));
	SDOManager->addSDO(new SDOSetPolePairNumber(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, deviceParams_->pole_pair_number));
	SDOManager->addSDO(new SDOSetThermalTimeConstantWinding(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, deviceParams_->thermal_time_constant_winding));

	SDOManager->addSDO(new SDOSetContinuousCurrentLimit(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, (int)(deviceParams_->continuous_current_limit*1000.0)));
	SDOManager->addSDO(new SDOSetOutputCurrentLimit(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, (int)(deviceParams_->continuous_current_limit*1000.0)));

	/* set gains */
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

	setPositionLimits(deviceParams_->positionLimits);


	// SDOManager->addSDO(new SDOSetOperationMode(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, deviceParams_->operationMode));
	SDOManager->addSDO(new SDOSetOperationMode(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, operation_mode_));

	// 1=Fault signal only instead of Quickstop
	SDOManager->addSDO(new SDOSetAbortConnectionOptionCode(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x01));
}

bool DeviceEPOS2Motor::resetDevice()
{
	SDOManager* SDOManager = bus_->getSDOManager();

	SDOManager->addSDO(new SDONMTResetNode(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	return true;
}

bool DeviceEPOS2Motor::initDevice(signed int operation_mode)
{
	SDOManager* SDOManager = bus_->getSDOManager();

	SDOManager->addSDO(new SDONMTEnterPreOperational(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	SDOManager->addSDO(new SDOSetCOBIDSYNC(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x80));

    operation_mode_ = operation_mode;
	configTxPDOs();
	configRxPDOs();

	setMotorParameters();
	initMotor();


	SDOManager->addSDO(new SDONMTStartRemoteNode(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	return true;

}

bool DeviceEPOS2Motor::setOperationMode(int op_mode)
{
    operation_mode_ = op_mode;
	SDOManager* SDOManager = bus_->getSDOManager();
    // reset message flags
    rxPDOVelocity_->setFlag(0);
    rxPDOPosition_->setFlag(0);
	SDOManager->addSDO(new SDOSetOperationMode(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, operation_mode_));
    return true;
}

void DeviceEPOS2Motor::configTxPDOs()
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

void DeviceEPOS2Motor::configRxPDOs()
{

	SDOManager* SDOManager = bus_->getSDOManager();

	/* deactivate all RxPDOs */
	SDOManager->addSDO(new SDORxPDO1SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));
	SDOManager->addSDO(new SDORxPDO2SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));
	SDOManager->addSDO(new SDORxPDO3SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));
	SDOManager->addSDO(new SDORxPDO4SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));

#if 0
    switch (operation_mode_) {
        case OPERATION_MODE_VELOCITY:
            printf("Device %04X configured in velocity\n",nodeId_);
            configRxPDOVelocity();
            //configRxPDOPositionLimits();
            break;
        case OPERATION_MODE_POSITION:
            printf("Device %04X configured in position\n",nodeId_);
            configRxPDOProfilePosition();
            //configRxPDOPositionLimits();
            break;
        default:
            // do not configure
            break;
    }
#else
    configRxPDOVelocity();
    configRxPDOProfilePosition();
    //configRxPDOPositionLimits();
#endif
}

void DeviceEPOS2Motor::configTxPDOPositionVelocity()
{
	SDOManager* SDOManager = bus_->getSDOManager();

	/* Transmit PDO 1 Parameter */

	///< configure COB-ID Transmit PDO 1
	SDOManager->addSDO(new SDOTxPDO1ConfigureCOBID(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	///< Set Transmission Type: SYNC 0x01
	SDOManager->addSDO(new SDOTxPDO1SetTransmissionType(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x01)); // SYNC
	///< Number of Mapped Application Objects
	SDOManager->addSDO(new SDOTxPDO1SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));
	///< Mapping "Position actual value"
	SDOManager->addSDO(new SDOTxPDO1SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x01, 0x60640020));
	///< Mapping "Velocity"
	SDOManager->addSDO(new SDOTxPDO1SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x02, 0x606C0020));
	///< Number of Mapped Application Objects
	SDOManager->addSDO(new SDOTxPDO1SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x02));
}

void DeviceEPOS2Motor::configTxPDOAnalogCurrent()
{
	SDOManager* SDOManager = bus_->getSDOManager();

	// Transmit PDO 2 Parameter
	///< configure COB-ID Transmit PDO 2
	SDOManager->addSDO(new SDOTxPDO2ConfigureCOBID(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	///< Set Transmission Type: SYNC 0x01
	SDOManager->addSDO(new SDOTxPDO2SetTransmissionType(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x01)); // SYNC
	///< Number of Mapped Application Objects
	SDOManager->addSDO(new SDOTxPDO2SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));
	///< Mapping "actual current value - works!"
	SDOManager->addSDO(new SDOTxPDO2SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x01, 0x60780010));

	///< Mapping "status word"
	SDOManager->addSDO(new SDOTxPDO2SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x02, 0x60410010));

	///< Mapping "Analog value"
	SDOManager->addSDO(new SDOTxPDO2SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x03, 0x207C0110));
	///< Mapping "Digital value"
	/*	SDOManager->addSDO(new SDOTxPDO2SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x02, 0x22000020));*/


	///< Number of Mapped Application Objects
	SDOManager->addSDO(new SDOTxPDO2SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x03));

}


void DeviceEPOS2Motor::configRxPDOVelocity()
{
	SDOManager* SDOManager = bus_->getSDOManager();

	/* Receive PDO 1 Parameter */
	///< Step 1: Configure COB-ID of the RxPDO 1
	SDOManager->addSDO(new SDORxPDO1ConfigureCOBID(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	///< Step 2: Set Transmission Type: SYNC 0x01
	SDOManager->addSDO(new SDORxPDO1SetTransmissionType(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x01)); // SYNC
	///< Step 3: Number of Mapped Application Objects
	SDOManager->addSDO(new SDORxPDO1SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));
	///< Step 4: Mapping Objects
	///< Mapping "Velocity Setting Value"
	SDOManager->addSDO(new SDORxPDO1SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x01, 0x206B0020));
	///< Mapping "Operation Mode"
	SDOManager->addSDO(new SDORxPDO1SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x02, 0x60600008));
	///< Step 5: Number of Mapped Application Objects
	SDOManager->addSDO(new SDORxPDO1SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x02));
}

void DeviceEPOS2Motor::configRxPDOProfilePosition()
{
	SDOManager* SDOManager = bus_->getSDOManager();

	/* Receive PDO 3 Parameter */
	///< Step 1: Configure COB-ID of the RxPDO 2
	SDOManager->addSDO(new SDORxPDO2ConfigureCOBID(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	///< Step 2: Set Transmission Type: SYNC 0x01
	SDOManager->addSDO(new SDORxPDO2SetTransmissionType(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x01)); // SYNC
	///< Step 3: Number of Mapped Application Objects
	SDOManager->addSDO(new SDORxPDO2SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));
	///< Step 4: Mapping Objects

	///< Mapping "Operation Mode"
	SDOManager->addSDO(new SDORxPDO2SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x01, 0x60600008));

	///< Mapping "Target position"
	SDOManager->addSDO(new SDORxPDO2SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x02, 0x607A0020));
	///< Mapping "Controlword"
	SDOManager->addSDO(new SDORxPDO2SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x03, 0x60400010));


	///< Step 5: Number of Mapped Application Objects
	SDOManager->addSDO(new SDORxPDO2SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x03));
}


/*void DeviceEPOS2Motor::configRxPDOPositionLimits()
{
	SDOManager* SDOManager = bus_->getSDOManager();

	 Receive PDO 2 Parameter
	///< Step 1: Configure COB-ID of the RxPDO 2
	SDOManager->addSDO(new SDORxPDO2ConfigureCOBID(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	///< Step 2: Set Transmission Type: SYNC 0x01
	SDOManager->addSDO(new SDORxPDO2SetTransmissionType(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x01)); // SYNC
	///< Step 3: Number of Mapped Application Objects
	SDOManager->addSDO(new SDORxPDO2SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x00));
	///< Step 4: Mapping Objects
	///< Mapping "Minimal position limit"
	SDOManager->addSDO(new SDORxPDO2SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x01, 0x607D0120)); // object: 0x607D0120 = {index: 0x607D, sub-index: 0x01, length: 0x20 (=32bit)}
	///< Mapping "Maximal position limit"
	SDOManager->addSDO(new SDORxPDO2SetMapping(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x02, 0x607D0220));
	///< Step 5: Number of Mapped Application Objects
	SDOManager->addSDO(new SDORxPDO2SetNumberOfMappedApplicationObjects(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_, 0x01));
}*/

void DeviceEPOS2Motor::initMotor()
{
	SDOManager* SDOManager = bus_->getSDOManager();
	SDOManager->addSDO(new SDOFaultReset(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	SDOManager->addSDO(new SDOShutdown(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	SDOManager->addSDO(new SDOSwitchOn(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	// SDOManager->addSDO(new SDOEnableOperation(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
    enabled_ = false;
}


void DeviceEPOS2Motor::setEnableMotor()
{
	SDOManager* SDOManager = bus_->getSDOManager();
	SDOManager->addSDO(new SDOFaultReset(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	SDOManager->addSDO(new SDOShutdown(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	SDOManager->addSDO(new SDOSwitchOn(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
	SDOManager->addSDO(new SDOEnableOperation(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));

    switch (operation_mode_) {
        case OPERATION_MODE_VELOCITY:
            break;
        case OPERATION_MODE_POSITION:
            rxPDOPosition_->enable();
            break;
        default:
            // do not configure
            break;
    }
    enabled_ = true;
}

void DeviceEPOS2Motor::setDisableMotor()
{
	SDOManager* SDOManager = bus_->getSDOManager();
	SDOManager->addSDO(new SDODisableOperation(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
    switch (operation_mode_) {
        case OPERATION_MODE_VELOCITY:
            break;
        case OPERATION_MODE_POSITION:
            rxPDOPosition_->disable();
            break;
        default:
            // do not configure
            break;
    }
    enabled_ = false;
}


bool DeviceEPOS2Motor::getIsMotorEnabled(bool &flag)
{
#if 0
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
            enabled_ = flag;
			return true;
		}
	}

	return false;
#else
    enabled_ = txPDOAnalogCurrent_->isEnabled();
    return true;
#endif

}

bool DeviceEPOS2Motor::getIsMotorDisabled(bool &flag)
{
#if 0
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
            enabled_ = flag;
			return true;
		}
	}

	return false;
#else
    enabled_ = txPDOAnalogCurrent_->isDisabled();
    return true;
#endif
}

bool DeviceEPOS2Motor::getAnalogInputOne(double& value)
{
	SDOManager* SDOManager = bus_->getSDOManager();
	sdoAnalogInputOne_->hasTimeOut();

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
			sdoAnalogInputOne_ = SDOGetAnalogInputOne::SDOGetAnalogInputOnePtr(new SDOGetAnalogInputOne(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
			return true;
		}
	}

	return false;
}

bool DeviceEPOS2Motor::getAnalogInputTwo(double& value)
{
	SDOManager* SDOManager = bus_->getSDOManager();

	if (!sdoAnalogInputTwo_->hasTimeOut()) {
		if (!sdoAnalogInputTwo_->getIsReceived()) {
			if (!sdoAnalogInputTwo_->getIsWaiting()) {
				if (!sdoAnalogInputTwo_->getIsQueuing()) {
					SDOManager->addSDO((SDOMsgPtr)sdoAnalogInputTwo_);
				}
			}
		} else {
			int val;
			sdoAnalogInputTwo_->getAnalog(val);
			value = (double) val;
			sdoAnalogInputTwo_.reset();
			sdoAnalogInputTwo_ = SDOGetAnalogInputTwo::SDOGetAnalogInputTwoPtr(new SDOGetAnalogInputTwo(deviceParams_->inSDOSMId_, deviceParams_->outSDOSMId_, nodeId_));
			return true;
		}
	}

	return false;
}

const TxPDOAnalogCurrent* DeviceEPOS2Motor::getStatus() const
{
	return txPDOAnalogCurrent_;
}

std::string DeviceEPOS2Motor::getStatusString() const
{
	unsigned int statusword = txPDOAnalogCurrent_->getStatusWord();
    std::string out = "[ ";


	if(statusword & (1<<STATUSWORD_OPERATION_ENABLE_BIT))
		out += "Enabled ";

	if(!(statusword & (1<<STATUSWORD_OPERATION_ENABLE_BIT)))
		out += "Disabled";

	if(statusword & (1<<STATUSWORD_SWITCHED_ON_BIT))
		out += "Switched on";

	if(statusword & (1<<STATUSWORD_VOLTAGE_ENABLE_BIT))
		out += "Voltage enabled";

	if(statusword & (1<<STATUSWORD_FAULT_BIT))
		out += "Fault";

	if(statusword & (1<<STATUSWORD_INTERNAL_LIMIT_ACTIVE_BIT))
		out += "Internal limit active";

	if(statusword & (1<<STATUSWORD_SWITCH_ON_DISABLE_BIT))
		out += "Switch on disable";

	if(statusword & (1<<STATUSWORD_QUICK_STOP_BIT))
		out += "Quick stop";


	return out + "]";

}

