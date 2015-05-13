/*!
 * @file 	SDOReadMsg.cpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 */

#include "libcanplusplus/SDOReadMsg.hpp"

SDOReadMsg::SDOReadMsg(int inSMID,
		int outSMID,
		int nodeId,
		int index,
		int subindex)
		:SDOMsg(inSMID, outSMID, nodeId),
		 index_(index),
		 subindex_(subindex)
{

	int Length[8] = {1, 1, 1, 1, 1, 1, 1, 1};
	int Value[8] = {0x40,
					(index_ & 0x00ff),
					(index_ & 0xff00)>>8,
					subindex_,
					0x00,
					0x00,
					0x00,
					0x00};
	outputMsg_->setLength(Length);
	outputMsg_->setValue(Value);
	outputMsg_->setFlag(1);
}

SDOReadMsg::~SDOReadMsg()
{

}

void SDOReadMsg::processReceivedMsg()
{

}

