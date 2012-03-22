/*!
 * @file 	Bus.hpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, bus
 *
 */

#ifndef BUS_HPP_
#define BUS_HPP_

#include "PDOManager.hpp"
#include "SDOManager.hpp"
#include "DeviceManager.hpp"


class Bus;
class DeviceManager;

//! A bus brings together the SDO manager, the PDO managers and the device manager
/*!
 * @ingroup robotCAN, bus
 */
class Bus {
public:
	/*! Constructor
	 * @param	index of the bus
	 */
	Bus(int iBus);

	//! Destructor
	virtual ~Bus();

	/*! Gets a reference to the PDO manager that sends the PDOs to the nodes
	 * @return	PDO manager
	 */
	PDOManager* getRxPDOManager();

	/*! Gets a reference to the PDO manager that receives the PDOs from the nodes
	 * @return	PDO manager
	 */
	PDOManager* getTxPDOManager();

	/*! Gets a reference to the SDO manager that sends and receives the SDOS to and from the nodes,
	 * respectively.
	 * @return	SDO manager
	 */
	SDOManager* getSDOManager();

	/*! Gets a reference to the device manager
	 * @return device manager
	 */
	DeviceManager* getDeviceManager();

private:
	//! PDO manager  that sends the PDOs to the nodes
	PDOManager* rxPDOManager_;
	//! PDO manager that receives the PDOs from the nodes
	PDOManager* txPDOManager_;

	//! SDO manager that sends and receives SDOs
	SDOManager* SDOManager_;

	//! device manager
	DeviceManager* deviceManager_;

	//! index of the bus
	int iBus_;
};

#endif /* BUS_HPP_ */
