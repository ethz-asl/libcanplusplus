/*!
 * @file 	DeviceELMOMotor.hpp
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
#include "PDOELMOMotor.hpp"
#include "SDOELMOMotor.hpp"

/* include parameters */
#include "DeviceELMOMotorParameters.hpp"


class Device;


//! EPOS2 Device
/*! This class configures and manages the EPOS2 70/10 that control the motors of StarlETH.
 * @ingroup robotCAN, device
 */
class DeviceELMOMotor: public Device {
public:

	/*! Constructor
	 * @param nodeId	CAN node ID
	 * @param deviceParams	parameter struct
	 */
	DeviceELMOMotor(int nodeId, DeviceELMOMotorParameters* deviceParams);

	//! Destructor
	virtual ~DeviceELMOMotor();

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

	/*! Configures the PDO to measure the analog signal and the current
	* Is invoked by configTxPDOs()
	*/
	virtual void configTxPDOAnalogCurrent();

	/*! Configures the PDO to send the profile velocity
	 * Is invoked by configRxPDOs()
	 */
	virtual void configRxPDOProfileVelocity();

	/*! Configures the PDO to send the profile position
	 * Is invoked by configRxPDOs()
	 */
	virtual void configRxPDOPosition();

	//! Adds the command PDOs to the bus manager
	virtual void addRxPDOs();

	//! Adds the receiving PDOs to the bus manager
	virtual void addTxPDOs();

	/* get information */

	/*! Gets the joint position [rad]
	 * @return	joint position [rad]
	 */
	double getPosition();

	/*! Gets the joint velocity [rad]
	 * @return	joint velocity [rad/s]
	 */
	double getVelocity();

	/*! Gets the reference to the device parameters
	 * @return reference to the device parameters
	 */
	DeviceELMOMotorParameters* getDeviceParams();

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
	 * @param velocity 		joint velocity [rad/s]
	 */
	void setVelocity(double velocity);

	void setPosition(double jointPosition_rad);

	/*! Gets the current
	 * @return current [A]
	 */
	double getCurrent();

	/*! Gets the analog signal
	 * @return
	 */
	double getAnalog();

	bool getAnalogInputOne(double& value);

	/*! Sets the position limits of the joint
	 * Converts the limits to motor position limits
	 * @param positionLimit_rad array[2] of first and second position limit [rad]
	 */
	void setPositionLimits(double* positionLimit_rad);

	//! Enable the EPOS
	void setEnableMotor();

	//! Disable the EPOS
	void setDisableMotor();



	//bool getPoti(double &value);

protected:
	//! PDO message to measure position and velocity of the motor
	TxPDOPositionVelocity* txPDOPositionVelocity_;

	//! PDO message to send motor velocity command
	RxPDOVelocity* rxPDOVelocity_;

	//! PDO message to send motor position command
	RxPDOPosition* rxPDOPosition_;

	RxPDOELMOBinaryInterpreterCmd* rxPDOELMOBinaryInterpreterCmd_;

	//! PDO message to measure analog signal and current
	TxPDOAnalogCurrent* txPDOAnalogCurrent_;

	//! device parameters
	DeviceELMOMotorParameters* deviceParams_;

	//! SDO to read the status word
	SDOReadStatusWord::SDOReadStatusWordPtr sdoStatusWord_;

	//! SDO to read if the EPOS is disabled
	SDOReadStatusWord::SDOReadStatusWordPtr sdoStatusWordDisabled_;

	//! SDO to read analog input
	SDOGetAnalogInputOne::SDOGetAnalogInputOnePtr sdoAnalogInputOne_;
};

#endif /* DEVICEEPOS2MOTOR_HPP_ */
