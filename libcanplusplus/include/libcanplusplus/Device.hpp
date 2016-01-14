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

#include <string>
#include "Bus.hpp"
#include "canopen_pdos.hpp"
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
	virtual void addRxPDOs() = 0;

	/*! Adds PDOs to the TxPDO manager
	 * This function is invoked by the device manager when this device is added.
	 */
	virtual void addTxPDOs() = 0;

	/*! Initialize the device (send SDOs to initialize it)
	 * @return true if successfully initialized
	 */
	virtual bool initDevice() = 0;

	/*! Initialize the heartbeat reception.
	 * This does NOT configure the heartbeat generation on the device. Do that manually in the initDevice function.
	 * @param heartBeatTime time in ms at which the producer sends heartbeat messages
	 * @return true if successfully initialized
	 */
	bool initHeartbeat(const unsigned int heartBeatTime);

	/*! Checks if the last heartbeat message has arrived recently
	 * @return true if within time window
	 */
	bool checkHeartbeat();

	virtual void sendNMTEnterPreOperational();
	virtual void sendNMTStartRemoteNode();
	virtual void setNMTRestartNode();

	const std::string& getName() const;
	void setName(const std::string& name);

protected:
	void sendSDO(SDOMsg* sdoMsg);
	bool checkSDOResponses(bool& success);

protected:
	//! the state the device is in
	enum class CANState : uint8_t {
		stopped = 0x4,
		operational = 0x5,
		preOperational = 0x7F
	};

	//!  reference to the CAN bus the device is connected to
	Bus* bus_;
	//! CAN node ID of device
	int nodeId_;

	std::string name_;

	//! List of SDO messages
	std::vector<SDOMsgPtr> sdos_;

	CANState canState_;

	//! Heartbeat time interval [ms]. Set to 0 to disable heartbeat message reception checking.
	uint16_t producerHeartBeatTime_;

	canopen::TxPDONMT* txPDONMT_;
};

#endif /* DEVICE_HPP_ */
