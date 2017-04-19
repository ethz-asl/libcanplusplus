/*!
 * @file 	Bus.cpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, bus
 *
 */

#include "libcanplusplus/Bus.hpp"

Bus::Bus(int iBus):iBus_(iBus)
{
	rxPDOManager_ = new PDOManager;
	txPDOManager_ = new PDOManager;
	SDOManager_ = new SDOManager(iBus);
	deviceManager_ = new DeviceManager(this);
}

Bus::~Bus()
{
	delete rxPDOManager_;
	delete txPDOManager_;
	delete SDOManager_;
	delete deviceManager_;
}
PDOManager* Bus::getRxPDOManager()
{
	return rxPDOManager_;
}

PDOManager* Bus::getTxPDOManager()
{
	return txPDOManager_;
}

SDOManager* Bus::getSDOManager()
{
	return SDOManager_;
}

DeviceManager* Bus::getDeviceManager()
{
	return deviceManager_;
}

int Bus::iBus()
{
	return iBus_;
}
