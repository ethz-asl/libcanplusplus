#pragma once

#include <libcanplusplus/SDOWriteMsg.hpp>
#include <libcanplusplus/SDOReadMsg.hpp>
#include <ros/ros.h>

#define WRITE_1_BYTE 0x2f
#define WRITE_2_BYTE 0x2b
#define WRITE_4_BYTE 0x23

namespace canopen {

//////////////////////////////////////////////////////////////////////////////
class SDOWrite: public SDOWriteMsg
{
public:
  SDOWrite(int inSDOSMId, int outSDOSMId, int nodeId, char length, int index, int subindex, int data):
    SDOWriteMsg(inSDOSMId, outSDOSMId, nodeId, length, index, subindex, data)
  {};
  virtual ~SDOWrite(){};
protected:
  virtual void processReceivedMsg()
  {
    if (inputMsg_->getValue()[1] == outputMsg_->getValue()[1] && inputMsg_->getValue()[2] == outputMsg_->getValue()[2]) {
      if (inputMsg_->getValue()[0] == 0x80)
      {
        ///< Check for recData[0]==0x60! recData[0]==0x80 means an error happend
        ROS_ERROR("SDO Error: Can't write! Error code: %02X%02X%02X%02X Output msg: COB ID: %04X Data: %02X %02X %02X %02X %02X %02X %02X %02X",
                      inputMsg_->getValue()[7], inputMsg_->getValue()[6], inputMsg_->getValue()[5], inputMsg_->getValue()[4],
                      outputMsg_->getCOBId(),
                      outputMsg_->getValue()[0], outputMsg_->getValue()[1], outputMsg_->getValue()[2], outputMsg_->getValue()[3], outputMsg_->getValue()[4], outputMsg_->getValue()[5], outputMsg_->getValue()[6], outputMsg_->getValue()[7]);
      }
    }
  }
};


//////////////////////////////////////////////////////////////////////////////
class SDORead: public SDOReadMsg
{
public:
  SDORead(int inSDOSMId, int outSDOSMId, int nodeId, int index, int subindex):
    SDOReadMsg(inSDOSMId, outSDOSMId, nodeId, index, subindex) //, isDataReceived_(false)
  {};
  virtual ~SDORead(){};

  virtual void setIsReceived(bool flag) {
    isReceived_ = flag;
  }
protected:

  int data_;

  virtual void processReceivedMsg()
  {

//    if (inputMsg_->getValue()[0] == 0x80)
//    {
//      ///< Check for recData[0]==0x60! recData[0]==0x80 means an error happend
//      printf("\e[0;31mAN ERROR HAPPEND. CAN'T WRITE CHECK CHAP. 12.3 of the firmware specification for the error code: %02X%02X%02X%02X\n\e[0m",inputMsg_->getValue()[7], inputMsg_->getValue()[6], inputMsg_->getValue()[5], inputMsg_->getValue()[4]);
//    }

    if (inputMsg_->getValue()[1] == outputMsg_->getValue()[1]
          && inputMsg_->getValue()[2] == outputMsg_->getValue()[2]
          && inputMsg_->getValue()[3] == outputMsg_->getValue()[3]) {
      if (inputMsg_->getValue()[0] == 0x80)
      {
        ///< Check for recData[0]==0x60! recData[0]==0x80 means an error happend
        printf("\e[0;31mAN ERROR HAPPEND. CAN'T WRITE CHECK CHAP. 12.3 of the firmware specification for the error code: %02X%02X%02X%02X\n\e[0m",inputMsg_->getValue()[7], inputMsg_->getValue()[6], inputMsg_->getValue()[5], inputMsg_->getValue()[4]);
      }
    }

    data_ = (inputMsg_->getValue()[4] + (inputMsg_->getValue()[5]<<8) + (inputMsg_->getValue()[6]<<16) + (inputMsg_->getValue()[7]<<24));

  }

};



/** *********************************************************************
----------------------------- NMT Server -------------------------------
********************************************************************* **/
//////////////////////////////////////////////////////////////////////////////
class SDONMTEnterPreOperational: public SDOWrite
{
public:
  SDONMTEnterPreOperational(int inSDOSMId, int outSDOSMId, int nodeId):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, 0, 0, 0, 0)
  {
    outputMsg_->setCOBId(0x00);
    inputMsg_->setCOBId(0x00);
    int Length[8] = {1, 1, 0, 0, 0, 0, 0, 0};
    int Value[8] = {0x80,
            nodeId_,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00};
    outputMsg_->setLength(Length);
    outputMsg_->setValue(Value);
    outputMsg_->setFlag(1);
    isReceived_ = true;

  };
  virtual ~SDONMTEnterPreOperational(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDONMTStartRemoteNode: public SDOWrite
{
public:
  SDONMTStartRemoteNode(int inSDOSMId, int outSDOSMId, int nodeId):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, 0, 0, 0, 0)
  {
    outputMsg_->setCOBId(0x00);
    inputMsg_->setCOBId(0x00);
    int Length[8] = {1, 1, 0, 0, 0, 0, 0, 0};
    int Value[8] = {0x01,
            nodeId_,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00};
    outputMsg_->setLength(Length);
    outputMsg_->setValue(Value);
    outputMsg_->setFlag(1);
    isReceived_ = true;
  };
  virtual ~SDONMTStartRemoteNode(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDONMTStopRemoteNode: public SDOWrite
{
public:
  SDONMTStopRemoteNode(int inSDOSMId, int outSDOSMId, int nodeId):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, 0, 0, 0, 0)
  {

    outputMsg_->setCOBId(0x00);
    inputMsg_->setCOBId(0x00);
    int Length[8] = {1, 1, 0, 0, 0, 0, 0, 0};
    int Value[8] = {0x02,
            nodeId,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00};
    outputMsg_->setLength(Length);
    outputMsg_->setValue(Value);
    outputMsg_->setFlag(1);
    isReceived_ = true;

  };
  virtual ~SDONMTStopRemoteNode(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDONMTResetCommunication: public SDOWrite
{
public:
  SDONMTResetCommunication(int inSDOSMId, int outSDOSMId, int nodeId):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, 0, 0, 0, 0)
  {
    outputMsg_->setCOBId(0x00);
    inputMsg_->setCOBId(0x00);
    int Length[8] = {1, 1, 0, 0, 0, 0, 0, 0};
    int Value[8] = {0x82,
            nodeId_,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00};
    outputMsg_->setLength(Length);
    outputMsg_->setValue(Value);
    outputMsg_->setFlag(1);
    isReceived_ = true;

  };
  virtual ~SDONMTResetCommunication(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDONMTResetNode: public SDOWrite
{
public:
  SDONMTResetNode(int inSDOSMId, int outSDOSMId, int nodeId):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, 0, 0, 0, 0)
  {
    outputMsg_->setCOBId(0x00);
    inputMsg_->setCOBId(0x00);
    int Length[8] = {1, 1, 0, 0, 0, 0, 0, 0};
    int Value[8] = {0x81,
            nodeId_,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00};
    outputMsg_->setLength(Length);
    outputMsg_->setValue(Value);
    outputMsg_->setFlag(1);
    isReceived_ = true;

  };
  virtual ~SDONMTResetNode(){};
};



/** *********************************************************************
----------------------------- Communication -----------------------------
********************************************************************* **/
//////////////////////////////////////////////////////////////////////////////
class SDOSetRS232Baudrate: public SDOWrite
{
public:
  SDOSetRS232Baudrate(int inSDOSMId, int outSDOSMId, int nodeId, int baudrate):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x2002, 0x00, baudrate)
  {};
  virtual ~SDOSetRS232Baudrate(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDOSetCANBitrate: public SDOWrite
{
public:
  SDOSetCANBitrate(int inSDOSMId, int outSDOSMId, int nodeId, int bitrate):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_2_BYTE, 0x2001, 0x00, bitrate)
  {};
  virtual ~SDOSetCANBitrate(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDOSetAbortConnectionOptionCode: public SDOWrite
{
public:
  SDOSetAbortConnectionOptionCode(int inSDOSMId, int outSDOSMId, int nodeId, int value):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_2_BYTE, 0x6007, 0x00, value)
  {};
  virtual ~SDOSetAbortConnectionOptionCode(){};
};


/** *********************************************************************
----------------------------- Initialization ----------------------------
********************************************************************* **/
//////////////////////////////////////////////////////////////////////////////
class SDOControlword: public SDOWrite
{
public:
  SDOControlword(int inSDOSMId, int outSDOSMId, int nodeId, int controlword):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_2_BYTE, 0x6040, 0x00, controlword)
  {};
  virtual ~SDOControlword(){};
};
//////////////////////////////////////////////////////////////////////////////
class SDOShutdown: public SDOWrite
{
public:
  SDOShutdown(int inSDOSMId, int outSDOSMId, int nodeId):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_2_BYTE, 0x6040, 0x00, 0x06)
  {};
  virtual ~SDOShutdown(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDOSwitchOn: public SDOWrite
{
public:
  SDOSwitchOn(int inSDOSMId, int outSDOSMId, int nodeId):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_2_BYTE, 0x6040, 0x00, 0x07)
  {};
  virtual ~SDOSwitchOn(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDOEnableOperation: public SDOWrite
{
public:
  SDOEnableOperation(int inSDOSMId, int outSDOSMId, int nodeId):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_2_BYTE, 0x6040, 0x00, 0x0F)
  {};
  virtual ~SDOEnableOperation(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDODisableOperation: public SDOWrite
{
public:
  SDODisableOperation(int inSDOSMId, int outSDOSMId, int nodeId):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_2_BYTE, 0x6040, 0x00, 0x07)
  {};
  virtual ~SDODisableOperation(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDOFaultReset: public SDOWrite
{
public:
  SDOFaultReset(int inSDOSMId, int outSDOSMId, int nodeId):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_2_BYTE, 0x6040, 0x00, 0x80)
  {};
  virtual ~SDOFaultReset(){};
};

/************************************************************************
------------------------ Digital Inputs ---------------------------------
********************************************************************* **/
//////////////////////////////////////////////////////////////////////////////
class SDOSetDigitalInputFunctionalitiesMask: public SDOWrite
{
public:
  SDOSetDigitalInputFunctionalitiesMask(int inSDOSMId, int outSDOSMId, int nodeId, int value):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_2_BYTE, 0x2071, 0x02, value)
  {};
  virtual ~SDOSetDigitalInputFunctionalitiesMask(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDOSetDigitalInputFunctionalitiesPolarity
: public SDOWrite
{
public:
  SDOSetDigitalInputFunctionalitiesPolarity(int inSDOSMId, int outSDOSMId, int nodeId, int value):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_2_BYTE, 0x2071, 0x03, value)
  {};
  virtual ~SDOSetDigitalInputFunctionalitiesPolarity(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDOSetDigitalInputFunctionalitiesExecutionMask
: public SDOWrite
{
public:
  SDOSetDigitalInputFunctionalitiesExecutionMask(int inSDOSMId, int outSDOSMId, int nodeId, int value):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_2_BYTE, 0x2071, 0x04, value)
  {};
  virtual ~SDOSetDigitalInputFunctionalitiesExecutionMask(){};
};


/***********************************************************************
-------------------- Life guard and heartbeat control-------------------
********************************************************************* **/
//////////////////////////////////////////////////////////////////////////////
class SDOSetGuardTime: public SDOWrite
{
public:
  SDOSetGuardTime(int inSDOSMId, int outSDOSMId, int nodeId, int time_ms):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_2_BYTE, 0x100C, 0x00, time_ms)
  {};
  virtual ~SDOSetGuardTime(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDOSetLifeTimeFactor: public SDOWrite
{
public:
  SDOSetLifeTimeFactor(int inSDOSMId, int outSDOSMId, int nodeId, int factor):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x100D, 0x00, factor)
  {};
  virtual ~SDOSetLifeTimeFactor(){};
};
//////////////////////////////////////////////////////////////////////////////
class SDOReadProducerHeartbeatTime: public SDORead
{
public:
  SDOReadProducerHeartbeatTime(int inSDOSMId, int outSDOSMId, int nodeId):
    SDORead(inSDOSMId, outSDOSMId, nodeId, 0x1017, 0x00),
    time_(0)
  {};


  virtual void processReceivedMsg()
  {
    SDORead::processReceivedMsg();
    time_ = (uint16_t)(inputMsg_->getValue()[0] + (inputMsg_->getValue()[1]<<8));
  }

  virtual ~SDOReadProducerHeartbeatTime(){};

  uint16_t getTime() const {
    return time_;
  }

private:
  uint16_t time_;
};

/***********************************************************************
------------------------------ Utilities --------------------------------
********************************************************************* **/
//////////////////////////////////////////////////////////////////////////////
class SDOSetCOBIDSYNC: public SDOWrite
{
public:
  SDOSetCOBIDSYNC(int inSDOSMId, int outSDOSMId, int nodeId, int ID):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1005, 0x00, ID)
  {};
  virtual ~SDOSetCOBIDSYNC(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDOSaveAllParameters: public SDOWrite
{
public:
  SDOSaveAllParameters(int inSDOSMId, int outSDOSMId, int nodeId):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1010, 0x01, 0x65766173)
  {};
  virtual ~SDOSaveAllParameters(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDORestoreAllDefaultParameters: public SDOWrite
{
public:
  SDORestoreAllDefaultParameters(int inSDOSMId, int outSDOSMId, int nodeId):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1011, 0x01, 0x64616F6C)
  {};
  virtual ~SDORestoreAllDefaultParameters(){};
};


/** *********************************************************************
------------------------------ Tx PDO's ---------------------------------
********************************************************************* **/

/*********************************************************************
 * PDO 1 Parameter
 *********************************************************************/
//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO1SetNumberOfMappedApplicationObjects: public SDOWrite
{
public:
  SDOTxPDO1SetNumberOfMappedApplicationObjects(int inSDOSMId, int outSDOSMId, int nodeId, int number_of_mapped_objects):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1A00, 0x00, number_of_mapped_objects)
  {};
  virtual ~SDOTxPDO1SetNumberOfMappedApplicationObjects(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO1ConfigureCOBID: public SDOWrite
{
public:
  SDOTxPDO1ConfigureCOBID(int inSDOSMId, int outSDOSMId, int nodeId):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1800, 0x01, 0x40000180 + nodeId)
  {};
  virtual ~SDOTxPDO1ConfigureCOBID(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO1SetTransmissionType: public SDOWrite
{
public:
  SDOTxPDO1SetTransmissionType(int inSDOSMId, int outSDOSMId, int nodeId, int type):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1800, 0x02, type)
  {};
  virtual ~SDOTxPDO1SetTransmissionType(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO1SetMapping: public SDOWrite
{
public:
  SDOTxPDO1SetMapping(int inSDOSMId, int outSDOSMId, int nodeId, int indexOfObject, int object):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1A00, indexOfObject, object)
  {};
  virtual ~SDOTxPDO1SetMapping(){};
};


/*********************************************************************
 * PDO 2 Parameter
 *********************************************************************/
//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO2SetNumberOfMappedApplicationObjects: public SDOWrite
{
public:
  SDOTxPDO2SetNumberOfMappedApplicationObjects(int inSDOSMId, int outSDOSMId, int nodeId, int number_of_mapped_objects):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1A01, 0x00, number_of_mapped_objects)
  {};
  virtual ~SDOTxPDO2SetNumberOfMappedApplicationObjects(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO2ConfigureCOBID: public SDOWrite
{
public:
  SDOTxPDO2ConfigureCOBID(int inSDOSMId, int outSDOSMId, int nodeId):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1801, 0x01, 0x40000280 + nodeId)
  {};
  virtual ~SDOTxPDO2ConfigureCOBID(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO2SetTransmissionType: public SDOWrite
{
public:
  SDOTxPDO2SetTransmissionType(int inSDOSMId, int outSDOSMId, int nodeId, int type):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1801, 0x02, type)
  {};
  virtual ~SDOTxPDO2SetTransmissionType(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO2SetMapping: public SDOWrite
{
public:
  SDOTxPDO2SetMapping(int inSDOSMId, int outSDOSMId, int nodeId, int indexOfObject, int object):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1A01, indexOfObject, object)
  {};
  virtual ~SDOTxPDO2SetMapping(){};
};

/*********************************************************************
 * PDO 3 Parameter
 *********************************************************************/
//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO3SetNumberOfMappedApplicationObjects: public SDOWrite
{
public:
  SDOTxPDO3SetNumberOfMappedApplicationObjects(int inSDOSMId, int outSDOSMId, int nodeId, int number_of_mapped_objects):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1A02, 0x00, number_of_mapped_objects)
  {};
  virtual ~SDOTxPDO3SetNumberOfMappedApplicationObjects(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO3ConfigureCOBID: public SDOWrite
{
public:
  SDOTxPDO3ConfigureCOBID(int inSDOSMId, int outSDOSMId, int nodeId):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1802, 0x01, 0x40000380 + nodeId)
  {};
  virtual ~SDOTxPDO3ConfigureCOBID(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO3SetTransmissionType: public SDOWrite
{
public:
  SDOTxPDO3SetTransmissionType(int inSDOSMId, int outSDOSMId, int nodeId, int type):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1802, 0x02, type)
  {};
  virtual ~SDOTxPDO3SetTransmissionType(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO3SetMapping: public SDOWrite
{
public:
  SDOTxPDO3SetMapping(int inSDOSMId, int outSDOSMId, int nodeId, int indexOfObject, int object):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1A02, indexOfObject, object)
  {};
  virtual ~SDOTxPDO3SetMapping(){};
};

/*********************************************************************
 * PDO 4 Parameter
 *********************************************************************/
//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO4SetNumberOfMappedApplicationObjects: public SDOWrite
{
public:
  SDOTxPDO4SetNumberOfMappedApplicationObjects(int inSDOSMId, int outSDOSMId, int nodeId, int number_of_mapped_objects):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1A03, 0x00, number_of_mapped_objects)
  {};
  virtual ~SDOTxPDO4SetNumberOfMappedApplicationObjects(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO4ConfigureCOBID: public SDOWrite
{
public:
  SDOTxPDO4ConfigureCOBID(int inSDOSMId, int outSDOSMId, int nodeId):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1803, 0x01, 0x40000480 + nodeId)
  {};
  virtual ~SDOTxPDO4ConfigureCOBID(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO4SetTransmissionType: public SDOWrite
{
public:
  SDOTxPDO4SetTransmissionType(int inSDOSMId, int outSDOSMId, int nodeId, int type):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1803, 0x02, type)
  {};
  virtual ~SDOTxPDO4SetTransmissionType(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO4SetMapping: public SDOWrite
{
public:
  SDOTxPDO4SetMapping(int inSDOSMId, int outSDOSMId, int nodeId, int indexOfObject, int object):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1A03, indexOfObject, object)
  {};
  virtual ~SDOTxPDO4SetMapping(){};
};


/** *********************************************************************
------------------------------ Rx PDO's ---------------------------------
********************************************************************* **/
/*********************************************************************
 * PDO 1 Parameter
 *********************************************************************/
//////////////////////////////////////////////////////////////////////////////
class SDORxPDO1SetNumberOfMappedApplicationObjects: public SDOWrite
{
public:
  SDORxPDO1SetNumberOfMappedApplicationObjects(int inSDOSMId, int outSDOSMId, int nodeId, int number_of_mapped_objects):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1600, 0x00, number_of_mapped_objects)
  {};
  virtual ~SDORxPDO1SetNumberOfMappedApplicationObjects(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO1ConfigureCOBID: public SDOWrite
{
public:
  SDORxPDO1ConfigureCOBID(int inSDOSMId, int outSDOSMId, int nodeId):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1400, 0x01, 0x40000200 + nodeId)
  {};
  virtual ~SDORxPDO1ConfigureCOBID(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO1SetTransmissionType: public SDOWrite
{
public:
  SDORxPDO1SetTransmissionType(int inSDOSMId, int outSDOSMId, int nodeId, int type):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1400, 0x02, type)
  {};
  virtual ~SDORxPDO1SetTransmissionType(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO1SetMapping: public SDOWrite
{
public:
  SDORxPDO1SetMapping(int inSDOSMId, int outSDOSMId, int nodeId, int indexOfObject, int object):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1600, indexOfObject, object)
  {};
  virtual ~SDORxPDO1SetMapping(){};
};

/*********************************************************************
 * PDO 2 Parameter
 *********************************************************************/
//////////////////////////////////////////////////////////////////////////////
class SDORxPDO2SetNumberOfMappedApplicationObjects: public SDOWrite
{
public:
  SDORxPDO2SetNumberOfMappedApplicationObjects(int inSDOSMId, int outSDOSMId, int nodeId, int number_of_mapped_objects):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1601, 0x00, number_of_mapped_objects)
  {};
  virtual ~SDORxPDO2SetNumberOfMappedApplicationObjects(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO2ConfigureCOBID: public SDOWrite
{
public:
  SDORxPDO2ConfigureCOBID(int inSDOSMId, int outSDOSMId, int nodeId):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1401, 0x01, 0x40000300 + nodeId)
  {};
  virtual ~SDORxPDO2ConfigureCOBID(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO2SetTransmissionType: public SDOWrite
{
public:
  SDORxPDO2SetTransmissionType(int inSDOSMId, int outSDOSMId, int nodeId, int type):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1401, 0x02, type)
  {};
  virtual ~SDORxPDO2SetTransmissionType(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO2SetMapping: public SDOWrite
{
public:
  SDORxPDO2SetMapping(int inSDOSMId, int outSDOSMId, int nodeId, int indexOfObject, int object):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1601, indexOfObject, object)
  {};
  virtual ~SDORxPDO2SetMapping(){};
};

/*********************************************************************
 * PDO 3 Parameter
 *********************************************************************/
//////////////////////////////////////////////////////////////////////////////
class SDORxPDO3SetNumberOfMappedApplicationObjects: public SDOWrite
{
public:
  SDORxPDO3SetNumberOfMappedApplicationObjects(int inSDOSMId, int outSDOSMId, int nodeId, int number_of_mapped_objects):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1602, 0x00, number_of_mapped_objects)
  {};
  virtual ~SDORxPDO3SetNumberOfMappedApplicationObjects(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO3ConfigureCOBID: public SDOWrite
{
public:
  SDORxPDO3ConfigureCOBID(int inSDOSMId, int outSDOSMId, int nodeId):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1402, 0x01, 0x40000400 + nodeId)
  {};
  virtual ~SDORxPDO3ConfigureCOBID(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO3SetTransmissionType: public SDOWrite
{
public:
  SDORxPDO3SetTransmissionType(int inSDOSMId, int outSDOSMId, int nodeId, int type):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1402, 0x02, type)
  {};
  virtual ~SDORxPDO3SetTransmissionType(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO3SetMapping: public SDOWrite
{
public:
  SDORxPDO3SetMapping(int inSDOSMId, int outSDOSMId, int nodeId, int indexOfObject, int object):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1602, indexOfObject, object)
  {};
  virtual ~SDORxPDO3SetMapping(){};
};

/*********************************************************************
 * PDO 4 Parameter
 *********************************************************************/
//////////////////////////////////////////////////////////////////////////////
class SDORxPDO4SetNumberOfMappedApplicationObjects: public SDOWrite
{
public:
  SDORxPDO4SetNumberOfMappedApplicationObjects(int inSDOSMId, int outSDOSMId, int nodeId, int number_of_mapped_objects):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1603, 0x00, number_of_mapped_objects)
  {};
  virtual ~SDORxPDO4SetNumberOfMappedApplicationObjects(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO4ConfigureCOBID: public SDOWrite
{
public:
  SDORxPDO4ConfigureCOBID(int inSDOSMId, int outSDOSMId, int nodeId):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1403, 0x01, 0x40000500 + nodeId)
  {};
  virtual ~SDORxPDO4ConfigureCOBID(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO4SetTransmissionType: public SDOWrite
{
public:
  SDORxPDO4SetTransmissionType(int inSDOSMId, int outSDOSMId, int nodeId, int type):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1403, 0x02, type)
  {};
  virtual ~SDORxPDO4SetTransmissionType(){};
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO4SetMapping: public SDOWrite
{
public:
  SDORxPDO4SetMapping(int inSDOSMId, int outSDOSMId, int nodeId, int indexOfObject, int object):
    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1603, indexOfObject, object)
  {};
  virtual ~SDORxPDO4SetMapping(){};
};



} // namespace
