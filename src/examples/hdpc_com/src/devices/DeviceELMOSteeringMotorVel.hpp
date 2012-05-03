/*!
 * @file 	DeviceELMOSteeringMotorVel.hpp
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, device
 *
 */

#ifndef DEVICEELMOSTEERINGMOTORVEL_HPP_
#define DEVICEELMOSTEERINGMOTORVEL_HPP_

#include "DeviceELMOBaseMotor.hpp"


//! ELMO steering Motor Device
/*! This class configures and the ELMOs that control the steering motors of the HDPC.
 * @ingroup robotCAN, device
 */
class DeviceELMOSteeringMotorVel: public DeviceELMOBaseMotor {
public:

	//! Used for homing procedure
	double absCurrentJointPosition_;

	/*! Constructor
	 * @param nodeId	CAN node ID
	 * @param deviceParams	parameter struct
	 */
	DeviceELMOSteeringMotorVel(int nodeId, DeviceELMOMotorParametersHDPC* deviceParams);

	//! Destructor
	virtual ~DeviceELMOSteeringMotorVel();

	/* configure at init */
	virtual void setMotorParameters();

	//! Initializes the EPOS
	virtual bool initDevice();

	/*! Configures all RxPDOs on the EPOS
	 * Is invoked by initDevice()
	 */
	virtual void configRxPDOs();

	/*! Configures the PDO to send the profile position
	 * Is invoked by configRxPDOs()
	 */
	virtual void configRxPDOProfileVelocity();

	//! Adds the command PDOs to the bus manager
	virtual void addRxPDOs();


	/* configure at run-time */

	/*! Sets the desired joint position [rad] with maximum velocity [rad/s]
	 * Use setProfileVelocity
	 * @param position 		motor position [rad]
	 * @param velocity 		motor velocity [rad/s]
	 */
    void setProfilePosition(double jointPosition_rad, double jointVelocity_rad_s);

	/*! Sets the desired joint velocity [rad/s]
	 * Converts the velocity to the desired motor velocity [rpm]
	 * @param velocity 		motor velocity [rad/s]
	 */
	virtual void setProfileVelocity(double velocity);


	//! Enable the ELMO
	virtual void setEnableMotor();

	//! Disable the ELMO
	virtual void setDisableMotor();


protected:
	//! PDO message to send motor velocity command
	RxPDOVelocity* rxPDOVelocity_;

};

#endif /* DEVICEELMOSTEERINGMOTORVEL_HPP_ */
