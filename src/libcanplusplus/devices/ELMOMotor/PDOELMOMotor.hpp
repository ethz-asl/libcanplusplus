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

#include "StatusWordBits.hpp"

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
	RxPDOVelocity(int nodeId, int SMId):CANOpenMsg(0x300+nodeId, SMId),
	isEnabled_(false)
	{
		//value_[2] = 0x000F;		///< Controlword (enable)
		value_[2] = 0x0007;		///< Controlword (disable)
		value_[0] = 0x03;		///< Profile Velocity Mode

		length_[0] = 1;			///< Profile Velocity Mode
		length_[1] = 4;			///< Target Position
		length_[2] = 2;			///< Controlword
	};

	virtual ~RxPDOVelocity() {};

	void setVelocity(int velocity)
	{
		if (isEnabled_) {
			value_[1] = velocity;
			flag_ = 1;
		}
	};

	void disable()
	{
		value_[1] = 0.0;
		value_[2] = 0x0007;		///< Controlword (disable)
		isEnabled_ = false;
		flag_ = 1;
	};

	void enable()
	{
		value_[1] = 0.0;
		value_[2] = 0x000F;		///< Controlword (enable)
		isEnabled_ = true;
		flag_ = 1;
	};

private:
	bool isEnabled_;
};

//////////////////////////////////////////////////////////////////////////////
class RxPDOPosition: public CANOpenMsg {
public:
	RxPDOPosition(int nodeId, int SMId):CANOpenMsg(0x400+nodeId, SMId),isOn_(true),isEnabled_(false)
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
class RxPDOExCmd: public CANOpenMsg {
public:
	RxPDOExCmd(int nodeId, int SMId):CANOpenMsg(0x300+nodeId, SMId)
	{
		value_[0] = 0x42;
		value_[1] = 0x47;
		value_[2] = 0x00;
		value_[3] = 0x00;

		length_[0] = 1;
		length_[1] = 1;
		length_[2] = 1;
		length_[3] = 1;
	};

	virtual ~RxPDOExCmd() {

	};

	void begin() {
		flag_ = 1;
	}

	void stop() {
		flag_ = 1;
	}
};
//////////////////////////////////////////////////////////////////////////////
class RxPDOELMOBinaryInterpreterCmd: public CANOpenMsg {
public:
	RxPDOELMOBinaryInterpreterCmd(int nodeId, const char* cmd, unsigned char index, int SMId):CANOpenMsg(0x300+nodeId, SMId)
	{
		flag_ = 1;
		value_[0] = cmd[0];
		value_[1] = cmd[1];
		value_[2] = index;
		value_[3] = 64; // float
		value_[4] = 0;
		value_[5] = 0;
		value_[6] = 0;
		value_[7] = 0;

		length_[0]=1;
		length_[1]=1;
		length_[2]=1;
		length_[3] = 1;
		length_[4] = 1;
		length_[5] = 1;
		length_[6] = 1;
		length_[7] = 1;


	};

	virtual ~RxPDOELMOBinaryInterpreterCmd() {

	};

};


//////////////////////////////////////////////////////////////////////////////
class TxPDOPositionVelocity: public CANOpenMsg {
public:
	TxPDOPositionVelocity(int nodeId, int SMId):CANOpenMsg(0x380+nodeId, SMId)
	{

	};

	virtual ~TxPDOPositionVelocity() {};

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

//////////////////////////////////////////////////////////////////////////////
class TxPDOAnalogCurrent: public CANOpenMsg {
public:
	TxPDOAnalogCurrent(int nodeId, int SMId):CANOpenMsg(0x480+nodeId, SMId)
	{

	};

	virtual ~TxPDOAnalogCurrent() {};

	virtual void processMsg()
	{

		short val;
		val = (value_[0] + (value_[1]<<8));
		analog_ = int(val);

		val = (value_[2] + (value_[3]<<8));
		current_ = int(val);

		statusword_ = (int)((unsigned short)(value_[4] + (value_[5]<<8)));
	};

	int getAnalog()
	{
		return analog_;
	};

	int getCurrent()
	{
		return current_;
	};

	int getStatusword()
	{
		return statusword_;
	};

	bool isEnabled()
	{
		return (statusword_ & (1<<STATUSWORD_OPERATION_ENABLE_BIT));
	};

	bool isDisabled()
	{
		return !(statusword_ & (1<<STATUSWORD_OPERATION_ENABLE_BIT));
	};

	bool isSwitchedOn()
	{
		return (statusword_ & (1<<STATUSWORD_SWITCHED_ON_BIT));
	};

	bool isSwitchedOff()
	{
		return (statusword_ & (1<<STATUSWORD_SWITCH_ON_DISABLE_BIT));
	};

	bool isVoltageEnabled()
	{
		return (statusword_ & (1<<STATUSWORD_VOLTAGE_ENABLE_BIT));
	};

	bool isFault()
	{
		return (statusword_ & (1<<STATUSWORD_FAULT_BIT));
	};

	bool isInternalLimitActive()
	{
		return (statusword_ & (1<<STATUSWORD_INTERNAL_LIMIT_ACTIVE_BIT));
	}



private:
	int analog_;
	int current_;
	int statusword_;

};


#endif /* CANPDOS_HPP_ */
