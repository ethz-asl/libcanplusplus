/*!
 * @file 	DeviceELMOBaseMotor.hpp
 * @brief	Base motor class for steering and driving motor
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, device
 *
 */

#ifndef DEVICEELMOBASEMOTOR_HPP_
#define DEVICEELMOBASEMOTOR_HPP_

#include "Device.hpp"
#include "PDOELMOMotor.hpp"
#include "SDOELMOMotor.hpp"

/* include parameters */
#include "DeviceELMOMotorParametersHDPC.hpp"


class Device;


//! ELMO Base Device
/*! This class configures and manages the ELMOs that control the driving motors of the HDPC.
 * @ingroup robotCAN, device
 */
class DeviceELMOBaseMotor: public Device {
public:

	/*! Constructor
	 * @param nodeId	CAN node ID
	 * @param deviceParams	parameter struct
	 */
	DeviceELMOBaseMotor(int nodeId, DeviceELMOMotorParametersHDPC* deviceParams);

	//! Destructor
	virtual ~DeviceELMOBaseMotor();

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

	//! Adds the command PDOs to the bus manager
	virtual void addRxPDOs();

	//! Adds the receiving PDOs to the bus manager
	virtual void addTxPDOs();

	/* get information */

	/*! Gets the joint position [rad]
	 * @return	joint position [rad/s]
	 */
	virtual double getPosition();

	/*! Gets the joint velocity [rad]
	 * @return	joint velocity [rad/s]
	 */
	virtual double getVelocity();

	/*! Gets the current
	 * @return current [mA]
	 */
	double getCurrent();

	/*! Gets the analog signal
	 * @return
	 */
	double getAnalog();

	/*! Gets the reference to the device parameters
	 * @return reference to the device parameters
	 */
	virtual DeviceELMOMotorParametersHDPC* getDeviceParams();

	/*! Sends a SDO to check if the EPOS is enabled.
	 * @param flag	true if EPOS is enabled
	 * @return true if a response is received
	 */
	virtual bool getIsMotorEnabled(bool &flag);

	/*! Sends a SDO to check if the EPOS is disabled.
	 * @param flag	true if EPOS is disabled
	 * @return true if a response is received
	 */
	virtual bool getIsMotorDisabled(bool &flag);

	/* configure at run-time */


	/*! Sets the position limits of the joint
	 * Converts the limits to motor position limits
	 * @param positionLimit_rad array[2] of first and second position limit [rad]
	 */
	virtual void setPositionLimits(double* positionLimit_rad);

	//! Enable the EPOS
	virtual void setEnableMotor();

	//! Disable the EPOS
	virtual void setDisableMotor();


protected:
	//! PDO message to measure position and velocity of the motor
	TxPDOPositionVelocity* txPDOPositionVelocity_;

	//! PDO message to measure analog signal and current
	TxPDOAnalogCurrent* txPDOAnalogCurrent_;

	//! device parameters
	DeviceELMOMotorParametersHDPC* deviceParams_;

	//! SDO to read the status word
	SDOReadStatusWord::SDOReadStatusWordPtr sdoStatusWord_;

	//! SDO to read if the EPOS is disabled
	SDOReadStatusWord::SDOReadStatusWordPtr sdoStatusWordDisabled_;

};

#endif /* DEVICEELMOBASEMOTOR_HPP_ */
