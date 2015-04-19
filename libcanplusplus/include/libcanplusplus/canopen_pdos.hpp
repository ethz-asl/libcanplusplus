/*
 * canopen_pdos.hpp
 *
 *  Created on: Apr 16, 2015
 *      Author: gech
 */

#pragma once

namespace canopen {

//////////////////////////////////////////////////////////////////////////////
class RxPDOSync: public CANOpenMsg {
public:
  RxPDOSync(int SMId):CANOpenMsg(0x80, SMId) {
    flag_ = 1;
  };
  virtual ~RxPDOSync() {};
};

} // namespace canopen
