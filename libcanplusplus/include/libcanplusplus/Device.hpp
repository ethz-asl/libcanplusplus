/*!
 * @file 	Device.hpp
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, device
 *
 */

#ifndef DEVICE_HPP_
#define DEVICE_HPP_

#include "Bus.hpp"
#include <string>

class Bus;


//! A device that is connected via CAN.
/*! This class functions as a base class.
 * It provides functions to add PDOs to the bus manager and
 * an initialization function.
 * @ingroup robotCAN, device
 */
class Device {
public:

	/*! Constructor
	 * @param nodeId	ID of CAN node
	 */
	Device(int nodeId);

  Device(int nodeId, const std::string& name);

	//! Destructor
	virtual ~Device();

	/*! Sets the reference to the CAN bus the device is connected to
	 * @param bus	reference to bus
	 */
	void setBus(Bus* bus);

	/*! Adds PDOs to the RxPDO manager
	 *  This function is invoked by the device manager when this device is added.
	 */
	virtual void addRxPDOs();

	/*! Adds PDOs to the TxPDO manager
	 * This function is invoked by the device manager when this device is added.
	 */
	virtual void addTxPDOs();

	/*! Initialize the device (send SDOs to initialize it)
	 * @return true if successfully initialized
	 */
	virtual bool initDevice();

  const std::string& getName() const;
  void setName(const std::string& name);



protected:
 void sendSDO(SDOMsg* sdoMsg);
 bool checkSDOResponses(bool& success);
protected:
	//!  reference to the CAN bus the device is connected to
	Bus* bus_;
	//! CAN node ID of device
	int nodeId_;

	std::string name_;

	//! List of SDO messages
  std::vector<SDOMsgPtr> sdos_;

};

#endif /* DEVICE_HPP_ */
