/*!
 * @file 	CANOpenMsg.hpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 */

#ifndef CANOpenMsg_HPP_
#define CANOpenMsg_HPP_

#include "CANSharedMemoryDecl.hpp"

//! General CANOpen message container
/*! The message can be either a sent or received message.
 * It can be either a SDO or PDO message.
 * This class converts a stack of data to a CAN message and vice versa.
 *
 * To send a message, the following information is needed:
 * 	COBId: the Communication Object Identifier
 * 	flag: if true, the CAN message will be sent
 * 	value[8]: a stack of max. 8 values, each element of the array represents maximal a 32bit value
 *  length[8]: the length of each value in the stack in bytes (0-4)
 *
 * As an example, a PDO with a velocity command of 4bytes and a operation mode of 1byte
 * needs to be sent to the node with ID 1:
 * 	COBId=0x200+1
 * 	flag=true
 * 	value[0] = velocity
 * 	value[1] = operation_mode
 * 	value[2-7] = 0
 * 	length[0] = 4
 * 	length[1] = 1
 * 	length[2-7] = 0
 *
 * The stack of values is converted into a stream of unsigned chars by the function
 * getCANMsg().
 *
 * @ingroup robotCAN
 */
class CANOpenMsg {
public:
	/*! Constructor
	 * @param	COBId	Communication Object Identifier
	 * @param	SMId	Shared Memory Identifier
	 */
	CANOpenMsg(int COBId, int SMId);

	//! Destructor
	virtual ~CANOpenMsg();

	/*! Converts the stack of values to a stream of unsigned chars.
	 * @param[out]	canDataDes struct of CAN message
	 */
	virtual void getCANMsg(CAN_BusDataDes *canDataDes);

	/*! Converts a stream of unsigned chars to a stack of values.
	 * @param[out]	canDataMeas struct of CAN message
	 */
	virtual void setCANMsg(CAN_BusDataMeas *canDataMeas);

	/*! Hook function that is invoked by setCANMsg()
	 *  Allows to process an incoming message
	 */
	virtual void processMsg();

	/*! Gets the Communication Object Identifier
	 *
	 * @return COBId
	 */
	int getCOBId();

	/*! Gets the Shared Memory Identifier
	 * @return SMId
	 */
	int getSMId();

	/*! Gets flag whether the message will be sent or the message is received
	 * @return
	 */
	int getFlag();

	/*! Gets the stack of values
	 *
	 * @return reference to value_[8]
	 */
	int* getValue();

	/*! Gets the lengths of the values in the stack
	 * @return reference to length
	 */
	int* getLength();

	/*! Sets the flag if the message needs to be sent
	 * @param flag	if true message is sent
	 */
	void setFlag(int flag);

	/*! Sets the stack of values
	 * @param value	 array of length 8
	 */
	void setValue(int* value);

	/*! Lengtsh of the values in the stack
	 * @param length array of length 8
	 */
	void setLength(int* length);

	/*! Sets the Communication Object Identifier
	 * @param COBId	Communication Object Identifier
	 */
	void setCOBId(int COBId);

protected:
	//! Communication Object Identifier
	int COBId_;

	//! Shared Memory Identifier
	int SMId_;

	//! if true, the message will be sent or the message is received
	int flag_;

	//! data of the CAN message
	int value_[8];

	//! the lengths of the values in the stack value_
	int length_[8];

};

#endif /* CANOpenMsg_HPP_ */
