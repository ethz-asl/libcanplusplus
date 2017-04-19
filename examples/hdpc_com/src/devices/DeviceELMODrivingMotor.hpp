/*!
 * @file 	DeviceELMODrivingMotor.hpp
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, device
 *
 */

#ifndef DEVICEELMODRIVINGMOTOR_HPP_
#define DEVICEELMODRIVINGMOTOR_HPP_

#include "DeviceELMOBaseMotor.hpp"

//! ELMO Driving Motor Device
/*! This class configures and manages the ELMOs that control the driving motors of the HDPC.
 * @ingroup robotCAN, device
 */
class DeviceELMODrivingMotor: public DeviceELMOBaseMotor {
public:

	/*! Constructor
	 * @param nodeId	CAN node ID
	 * @param deviceParams	parameter struct
	 */
	DeviceELMODrivingMotor(int nodeId, DeviceELMOMotorParametersHDPC* deviceParams);

	//! Destructor
	virtual ~DeviceELMODrivingMotor();

	/* configure at init */
	virtual void setMotorParameters();

	//! Initializes the EPOS
	virtual bool initDevice();


	/*! Configures all RxPDOs on the EPOS
	 * Is invoked by initDevice()
	 */
	virtual void configRxPDOs();


	/*! Configures the PDO to send the profile velocity
	 * Is invoked by configRxPDOs()
	 */
	virtual void configRxPDOProfileVelocity();

	//! Adds the command PDOs to the bus manager
	virtual void addRxPDOs();



	/* get information */


	/* configure at run-time */

	/*! Sets the desired joint velocity [rad/s]
	 * Converts the velocity to the desired motor velocity [rpm]
	 * @param velocity 		motor velocity [rad/s]
	 */
	virtual void setProfileVelocity(double velocity);


	//! Enable the EPOS
	virtual void setEnableMotor();

	//! Disable the EPOS
	virtual void setDisableMotor();


protected:
	//! PDO message to send motor velocity command
	RxPDOVelocity* rxPDOVelocity_;

};

#endif /* DEVICEELMODRIVINGMOTOR_HPP_ */
