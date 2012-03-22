/*!
 * @file 	Device.cpp
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, device
 *
 */

#include "Device.hpp"
#include <stdio.h>

Device::Device(int nodeId)
:nodeId_(nodeId)
{

}

Device::~Device()
{

}

void Device::setBus(Bus* bus)
{
	bus_ = bus;
}

void Device::addRxPDOs()
{
	printf("Warning: Device::addRxPDOs is not implemented!\n");
}

void Device::addTxPDOs()
{
	printf("Warning: Device::addTxPDOs is not implemented!\n");
}


bool Device::initDevice()
{
	printf("Warning: Device::initDevice is not implemented!\n");
	return false;
}
