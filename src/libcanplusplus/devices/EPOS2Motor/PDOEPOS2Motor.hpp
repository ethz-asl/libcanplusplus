/*!
 * @file 	PDOEPOS2Motor.cpp
 * @brief	PDOs used for StarlETH
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 * TxPDO1 180
 * TxPDO2 280
 * TxPDO3 380
 * TxPDO4 480
 * RxPDO1 200
 * RxPDO2 300
 * RxPDO3 400
 * RxPDO4 500
 *
 */

#ifndef CANPDOS_HPP_
#define CANPDOS_HPP_


#include "CANOpenMsg.hpp"
#include <stdio.h>

//////////////////////////////////////////////////////////////////////////////
class RxPDOSync: public CANOpenMsg {
public:
	RxPDOSync(int SMId):CANOpenMsg(0x80, SMId) {
		flag_ = 1;
	};
	virtual ~RxPDOSync() {};
};

//////////////////////////////////////////////////////////////////////////////
class RxPDOVelocity: public CANOpenMsg {
public:
	RxPDOVelocity(int nodeId, int SMId):CANOpenMsg(0x200+nodeId, SMId)
	{
		value_[1] = 0xFE;		///< Velocity Mode = -2

		length_[0] = 4;			///< Target Velocity
		length_[1] = 1;			///<  Velocity Mode
		length_[2] = 0;			///< Controlword
	};

	virtual ~RxPDOVelocity() {
//		printf("~RxPDOVelocity()\n");
	};

	void setVelocity(int velocity)
	{
		value_[0] = velocity;
		flag_ = 1;
	};
};

//////////////////////////////////////////////////////////////////////////////
class RxPDOPosition: public CANOpenMsg {
public:
	RxPDOPosition(int nodeId, int SMId):CANOpenMsg(0x300+nodeId, SMId),isOn_(true),isEnabled_(false)
	{
		value_[2] = 0x003F;		///< Controlword 0x003F
		value_[0] = 0x01;		///< Profile Position Mode
		//value_[3] = 60000;
		length_[0] = 1;			///< Profile Position Mode
		length_[1] = 4;			///< Target Position
		length_[2] = 2;			///< Controlword
		//length_[3] = 4;
	};

	virtual ~RxPDOPosition() {};

	void setPosition(int position)
	{
		if (isEnabled_) {
			value_[1] = position;
			//value_[1] = 0x003F;		///< Controlword 0x003F
			flag_ = 1;
			if (isOn_) {
				value_[2] = 0x003F;
				isOn_ = false;
			} else {
				value_[2] = 0x002F;
				isOn_ = true;
			}
		}
	};

	void disable()
	{
		value_[2] = 0x0007;		///< Controlword (disable)
		isEnabled_ = false;
		flag_ = 1;
	};

	void enable()
	{
		value_[2] = 0x000F;		///< Controlword (enable)
		isEnabled_ = true;
		flag_ = 1;
	};

private:
	bool isOn_;
	bool isEnabled_;
};


//////////////////////////////////////////////////////////////////////////////
class TxPDOPositionVelocity: public CANOpenMsg {
public:
	TxPDOPositionVelocity(int nodeId, int SMId):CANOpenMsg(0x180+nodeId, SMId)
	{

	};

	virtual ~TxPDOPositionVelocity()
	{
//		printf("~TxPDOPositionVelocity()\n");
	};

	virtual void processMsg()
	{
		position_ = (value_[0] + (value_[1]<<8) + (value_[2]<<16) + (value_[3]<<24));
		velocity_ = (value_[4] + (value_[5]<<8) + (value_[6]<<16) + (value_[7]<<24));
	};

	int getPosition()
	{
		return position_;
	};

	int getVelocity()
	{
		return velocity_;
	};


private:
	int position_;
	int velocity_;

};
#endif /* CANPDOS_HPP_ */
