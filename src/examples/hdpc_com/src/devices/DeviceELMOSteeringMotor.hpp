/*!
 * @file 	DeviceELMOSteeringMotor.hpp
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, device
 *
 */

#ifndef DEVICEELMOSTEERINGMOTOR_HPP_
#define DEVICEELMOSTEERINGMOTOR_HPP_

#include "DeviceELMOBaseMotor.hpp"


//! ELMO steering Motor Device
/*! This class configures and the ELMOs that control the steering motors of the HDPC.
 * @ingroup robotCAN, device
 */
class DeviceELMOSteeringMotor: public DeviceELMOBaseMotor {
public:

	//! Used for homing procedure
	double absCurrentJointPosition_;

	/*! Constructor
	 * @param nodeId	CAN node ID
	 * @param deviceParams	parameter struct
	 */
	DeviceELMOSteeringMotor(int nodeId, DeviceELMOMotorParametersHDPC* deviceParams);

	//! Destructor
	virtual ~DeviceELMOSteeringMotor();

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
	virtual void configRxPDOProfilePosition();

	//! Adds the command PDOs to the bus manager
	virtual void addRxPDOs();


	/* configure at run-time */

	/*! Sets the desired joint position [rad]
	 * Converts the joint position to the desired motor position [ticks]
	 * @param position 		joint position [rad]
	 */
	void setProfilePosition(double jointPosition_rad);

	//! Enable the EPOS
	virtual void setEnableMotor();

	//! Disable the EPOS
	virtual void setDisableMotor();


	virtual void initMotor();



	void setHomeOffset(double jointPosition_rad);

protected:
	//! PDO message to send motor position command
	RxPDOPosition* rxPDOPosition_;

};

#endif /* DEVICEELMOSTEERINGMOTOR_HPP_ */
