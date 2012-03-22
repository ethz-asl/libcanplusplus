/*!
 * @file 	DeviceEPOS2Motor.hpp
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, device
 *
 */

#ifndef DEVICEEPOS2MOTOR_HPP_
#define DEVICEEPOS2MOTOR_HPP_

#include "Device.hpp"
#include "PDOEPOS2Motor.hpp"
#include "SDOEPOS2Motor.hpp"

/* include parameters */
#include "DeviceEPOS2MotorParameters.hpp"


class Device;


//! EPOS2 Device
/*! This class configures and manages the EPOS2 70/10 that control the motors of StarlETH.
 * @ingroup robotCAN, device
 */
class DeviceEPOS2Motor: public Device {
public:

	/*! Constructor
	 * @param nodeId	CAN node ID
	 * @param deviceParams	parameter struct
	 */
	DeviceEPOS2Motor(int nodeId, DeviceEPOS2MotorParameters* deviceParams);

	//! Destructor
	virtual ~DeviceEPOS2Motor();

	/* configure at init */
	virtual void setMotorParameters();

	//! Initializes the EPOS
	virtual bool initDevice();

	/*! Enables the EPOS
	 * Is invoked by initDevice();
	 */
	virtual void initMotor();

	/*! Configures all TxPDOs on the EPOS
	 * Is invoked by initDevice()
	 */
	virtual void configTxPDOs();

	/*! Configures all RxPDOs on the EPOS
	 * Is invoked by initDevice()
	 */
	virtual void configRxPDOs();

	/*! Configures the PDO to measure the position and velocity
	 * Is invoked by configTxPDOs()
	 */
	virtual void configTxPDOPositionVelocity();

	/*! Configures the PDO to send the command velocity
	 * Is invoked by configRxPDOs()
	 */
	virtual void configRxPDOVelocity();

	//! Adds the command PDOs to the bus manager
	virtual void addRxPDOs();

	//! Adds the receiving PDOs to the bus manager
	virtual void addTxPDOs();

	/* get information */

	/*! Gets the joint position [rad]
	 * @return	joint position [rad/s]
	 */
	double getPosition();

	/*! Gets the joint velocity [rad]
	 * @return	joint velocity [rad/s]
	 */
	double getVelocity();

	/*! Gets the reference to the device parameters
	 * @return reference to the device parameters
	 */
	DeviceEPOS2MotorParameters* getDeviceParams();

	/*! Sends a SDO to check if the EPOS is enabled.
	 * @param flag	true if EPOS is enabled
	 * @return true if a response is received
	 */
	bool getIsMotorEnabled(bool &flag);

	/*! Sends a SDO to check if the EPOS is disabled.
	 * @param flag	true if EPOS is disabled
	 * @return true if a response is received
	 */
	bool getIsMotorDisabled(bool &flag);

	/* configure at run-time */

	/*! Sets the desired joint velocity [rad/s]
	 * Converts the velocity to the desired motor velocity [rpm]
	 * @param velocity 		motor velocity [rad/s]
	 */
	void setVelocity(double velocity);

	/*! Sets the position limits of the joint
	 * Converts the limits to motor position limits
	 * @param positionLimit_rad array[2] of first and second position limit [rad]
	 */
	void setPositionLimits(double* positionLimit_rad);

	//! Enable the EPOS
	void setEnableMotor();

	//! Disable the EPOS
	void setDisableMotor();


protected:
	//! PDO message to measure position and velocity of the motor
	TxPDOPositionVelocity* txPDOPositionVelocity_;

	//! PDO message to send motor velocity command
	RxPDOVelocity* rxPDOVelocity_;

	//! device parameters
	DeviceEPOS2MotorParameters* deviceParams_;

	//! SDO to read the status word
	SDOReadStatusWord::SDOReadStatusWordPtr sdoStatusWord_;

	//! SDO to read if the EPOS is disabled
	SDOReadStatusWord::SDOReadStatusWordPtr sdoStatusWordDisabled_;
};

#endif /* DEVICEEPOS2MOTOR_HPP_ */
