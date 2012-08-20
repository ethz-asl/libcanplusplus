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

	//! Reset the EPOS
	virtual bool resetDevice();

	//! Initializes the EPOS
	virtual bool initDevice(signed int operation_mode = OPERATION_MODE_VELOCITY);

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
	virtual void configRxPDOs(signed int operation_mode = OPERATION_MODE_VELOCITY);

	/*! Configures the PDO to measure the position and velocity
	 * Is invoked by configTxPDOs()
	 */
	virtual void configTxPDOPositionVelocity();

	/*! Configures the PDO to send the command velocity
	 * Is invoked by configRxPDOs()
	 */
	virtual void configRxPDOVelocity();

	/*! Configures the PDO to send the command position
	 * Is invoked by configRxPDOs()
	 */
	virtual void configRxPDOProfilePosition();

	/*! Configures the PDO to send the position limits
	 * Is invoked by configRxPDOs()
	 */
	//virtual void configRxPDOPositionLimits();

	//! Adds the command PDOs to the bus manager
	virtual void addRxPDOs();

	//! Adds the receiving PDOs to the bus manager
	virtual void addTxPDOs();

	/* get information */

	/*! Gets the joint position [rad]
	 * @return	joint position [rad]
	 */
	double getPosition();

	/*! Gets the joint velocity [rad/s]
	 * @return	joint velocity [rad/s]
	 */
	double getVelocity();

	/*! Gets the reference to the device parameters
	 * @return reference to the device parameters
	 */
	DeviceEPOS2MotorParameters* getDeviceParams();

    /*! Returns the value of the internal enabled flag
     * Might not be reflecting the current state of the motor, but does not
     * cost an SDO. Use getIsMotorDisabled and getIsMotorEnabled to 
     * retrieve the real state
     * */
    bool isEnabled() const {return enabled_;}

    /*! Returns the value of the internal operation mode 
     * Might not be reflecting the current state of the motor, but does not
     * cost an SDO. 
     * */
    signed int getOperationMode() const {return operation_mode_;}

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

	/*! Sets the desired joint position [rad]
	 * Converts the position to the desired motor position [ticks]
	 * @param jointPosition_rad 	motor position [rad]
	 */
    void setPosition(double jointPosition_rad);

	/*! Sets the position limits of the joint per PDO
	 * Converts the limits to motor position limits
	 * @param positionLimit_rad array[2] of first and second position limit [rad]
	 */
	//void setPositionLimitsPerPDO(double* positionLimit_rad);

	/*! Sets the position limits of the joint
	 * Converts the limits to motor position limits
	 * @param positionLimit_rad array[2] of first and second position limit [rad]
	 */
	void setPositionLimits(double* positionLimit_rad);

	bool getAnalogInputOne(double& value);

	bool getAnalogInputTwo(double& value);

	//! Enable the EPOS
	void setEnableMotor();

	//! Disable the EPOS
	void setDisableMotor();


protected:
    //! Internal record of the motor state. Valid only until some external
    //factor brings the motor to a fault state
    bool enabled_;
    
    //! Internal record of the EPOS operation mode. Should be one of the
    //OPERATION_MODE_... constants. Only once can be active at a given time.
    signed int operation_mode_;

	//! PDO message to measure position and velocity of the motor
	TxPDOPositionVelocity* txPDOPositionVelocity_;

	//! PDO message to send motor velocity command
	RxPDOVelocity* rxPDOVelocity_;

	//! PDO message to send motor position command
	RxPDOPosition* rxPDOPosition_;

	//! PDO message to send motor position limits command
	//RxPDOPositionLimit* rxPDOPositionLimit_;

	//! device parameters
	DeviceEPOS2MotorParameters* deviceParams_;

	//! SDO to read the status word
	SDOReadStatusWord::SDOReadStatusWordPtr sdoStatusWord_;

	//! SDO to read if the EPOS is disabled
	SDOReadStatusWord::SDOReadStatusWordPtr sdoStatusWordDisabled_;

	//! SDO to read analog input 1
	SDOGetAnalogInputOne::SDOGetAnalogInputOnePtr sdoAnalogInputOne_;

	//! SDO to read analog input 2
	SDOGetAnalogInputTwo::SDOGetAnalogInputTwoPtr sdoAnalogInputTwo_;
};

#endif /* DEVICEEPOS2MOTOR_HPP_ */
