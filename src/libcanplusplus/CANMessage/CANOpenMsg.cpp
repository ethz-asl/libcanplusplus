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
#include "CANOpenMsg.hpp"


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


void CANOpenMsg::getCANMsg(CAN_BusDataDes *canDataDes)
{
	int k = 0;
	canDataDes->length = 0;
	for(int l=0; l<8; l++) {
		canDataDes->length = canDataDes->length + length_[l];
		assert(canDataDes->length<=8);
		for(int j=0; j<length_[l]; j++) {
			assert(k<8);
			canDataDes->value[k] = ((value_[l]>>(8*j)) & 0x000000ff);
			k++;
		}
	}
	canDataDes->COBId = COBId_;
	canDataDes->flag = flag_;
	canDataDes->rtr = rtr_;
}

void CANOpenMsg::setCANMsg(CAN_BusDataMeas *canDataMeas)
{
	length_[0] = canDataMeas->length;
	for(int i=0; i<canDataMeas->length; i++)
	{
		value_[i] = canDataMeas->value[i];
	}
	COBId_ = canDataMeas->COBId;
	flag_ = 1;
	rtr_ = canDataMeas->rtr;

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
