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


//! EPOS2 Device
/*! This class configures and manages the EPOS2 70/10 that control the motors of StarlETH.
 * @ingroup robotCAN, device
 */
class DeviceELMOSteeringMotor: public DeviceELMOBaseMotor {
public:

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


protected:
	//! PDO message to send motor position command
	RxPDOPosition* rxPDOPosition_;

};

#endif /* DEVICEELMOSTEERINGMOTOR_HPP_ */
