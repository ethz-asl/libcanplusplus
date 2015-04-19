/*!
 * @file 	CANOpenMsg.cpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 */

#include <assert.h>
#include "libcanplusplus/CANOpenMsg.hpp"


CANOpenMsg::CANOpenMsg(int COBId, int SMId)
:COBId_(COBId),
 SMId_(SMId),
 flag_(0),
 rtr_(0)
{
	for (int k=0;k<8; k++) {
		value_[k] = 0;
		length_[k] = 0;
	}
}


CANOpenMsg::~CANOpenMsg()
{

}


void CANOpenMsg::getCANMsg(CANMsg *transmitMessage)
{
	int k = 0;
	transmitMessage->length = 0;
	for(int l=0; l<8; l++) {
		transmitMessage->length = transmitMessage->length + length_[l];
		assert(transmitMessage->length<=8);
		for(int j=0; j<length_[l]; j++) {
			assert(k<8);
			transmitMessage->value[k] = ((value_[l]>>(8*j)) & 0x000000ff);
			k++;
		}
	}
	transmitMessage->COBId = COBId_;
	transmitMessage->flag = flag_;
	transmitMessage->rtr = rtr_;
}

void CANOpenMsg::setCANMsg(CANMsg *receiveMeassage)
{
	length_[0] = receiveMeassage->length;
	for(int i=0; i<receiveMeassage->length; i++)
	{
		value_[i] = receiveMeassage->value[i];
	}
	COBId_ = receiveMeassage->COBId;
	flag_ = 1;
	rtr_ = receiveMeassage->rtr;

	// Hook to process the message
	processMsg();
}

void CANOpenMsg::processMsg()
{

}

int CANOpenMsg::getCOBId()
{
	return COBId_;
}

int CANOpenMsg::getSMId()
{
	return SMId_;
}

int CANOpenMsg::getFlag()
{
	return flag_;
}

int CANOpenMsg::getRTR()
{
	return rtr_;
}

int* CANOpenMsg::getValue()
{
	return value_;
}

int* CANOpenMsg::getLength()
{
	return length_;
}

void CANOpenMsg::setFlag(int flag)
{
	flag_ = flag;
}

void CANOpenMsg::setRTR(int rtr)
{
	rtr_ = rtr;
}

void CANOpenMsg::setCOBId(int COBId)
{
	COBId_ = COBId;
}


void CANOpenMsg::setValue(int* value)
{
	for (int k=0; k<8; k++) {
		value_[k] = value[k];
	}
}

void CANOpenMsg::setLength(int* length)
{
	for (int k=0; k<8; k++) {
		length_[k] = length[k];
	}
}
