/*!
 * @file 	Device.cpp
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, device
 *
 */

#include "libcanplusplus/Device.hpp"
#include <stdio.h>

Device::Device(int nodeId, const std::string& name)
:nodeId_(nodeId),
 name_(name)
{

}

Device::Device(int nodeId)
:nodeId_(nodeId),
 name_()
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

void Device::sendSDO(SDOMsg* sdoMsg) {
  SDOMsgPtr sdo(sdoMsg);
  SDOManager* SDOManager = bus_->getSDOManager();
  sdos_.push_back(sdo);
  SDOManager->addSDO(sdo);
}

const std::string& Device::getName() const {
  return name_;
}

void Device::setName(const std::string& name) {
  name_ = name;
}

bool Device::checkSDOResponses(bool& success) {
  // Check if SDOS are processed
  bool done = true;
  // If one of the SDOs could not be sent, this flag will be false:
  success = true;

  for (auto& sdo : sdos_) {
    // Check if SDO was received or has a timeout
    if (!sdo->getIsReceived() &&  !sdo->hasTimeOut()) {
      done = false;
    }
    if(sdo->hasTimeOut()) {
      success = false;
    }
  }
  if (done) {
    sdos_.clear();
  }
  return done;
}
