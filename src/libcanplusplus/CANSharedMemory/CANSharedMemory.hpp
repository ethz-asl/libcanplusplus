/*!
 * @file 	CANSharedMemory.hpp
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 */
#ifndef CANSHAREDMEMORY_HPP_
#define CANSHAREDMEMORY_HPP_


#include "CANSharedMemoryDecl.hpp"
#ifdef USE_SL_FUNCTIONS
#include "SL_vx_wrappers.h"
#else
	#include "SLVxWrapper.hpp"
#endif


typedef struct smCANBusDataDes {
  SEM_ID          sm_sem;
  float           ts;
  CAN_BusDataDes  busData[1];
} smCANBusDataDes;


typedef struct smCANBusDataMeas {
  SEM_ID          sm_sem;
  float           ts;
  CAN_BusDataMeas  busData[1];
} smCANBusDataMeas;


#ifdef USE_XSENS
typedef struct smXSENSDataMeas {
  SEM_ID          sm_sem;
  float           ts;
  XSENS_DataMeas  data[1];
} smXSENSDataMeas;
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* external variables */
extern SEM_ID             sm_canbusdata_des_one_ready_sem;
extern SEM_ID             sm_canbusdata_des_two_ready_sem;
extern SEM_ID             sm_canbusdata_des_three_ready_sem;
extern SEM_ID             sm_canbusdata_des_four_ready_sem;

extern SEM_ID             sm_canbusdata_meas_one_ready_sem;
extern SEM_ID             sm_canbusdata_meas_two_ready_sem;
extern SEM_ID             sm_canbusdata_meas_three_ready_sem;
extern SEM_ID             sm_canbusdata_meas_four_ready_sem;


extern SEM_ID             sm_canbus_init_one_sem;
extern SEM_ID             sm_canbus_init_two_sem;
extern SEM_ID             sm_canbus_init_three_sem;
extern SEM_ID             sm_canbus_init_four_sem;

extern SEM_ID             sm_canbus_wait_one_sem;
extern SEM_ID             sm_canbus_wait_two_sem;
extern SEM_ID             sm_canbus_wait_three_sem;
extern SEM_ID             sm_canbus_wait_four_sem;


extern smCANBusDataDes 	*sm_canbusdata_des_one;
extern SEM_ID            sm_canbusdata_des_one_sem;
extern CAN_BusDataDes   *sm_canbusdata_des_one_data;

extern smCANBusDataMeas *sm_canbusdata_meas_one;
extern SEM_ID            sm_canbusdata_meas_one_sem;
extern CAN_BusDataMeas  *sm_canbusdata_meas_one_data;


extern smCANBusDataDes 	*sm_canbusdata_des_two;
extern SEM_ID            sm_canbusdata_des_two_sem;
extern CAN_BusDataDes   *sm_canbusdata_des_two_data;

extern smCANBusDataMeas *sm_canbusdata_meas_two;
extern SEM_ID            sm_canbusdata_meas_two_sem;
extern CAN_BusDataMeas  *sm_canbusdata_meas_two_data;

extern smCANBusDataDes  *sm_canbusdata_des_three;
extern SEM_ID            sm_canbusdata_des_three_sem;
extern CAN_BusDataDes   *sm_canbusdata_des_three_data;

extern smCANBusDataMeas *sm_canbusdata_meas_three;
extern SEM_ID            sm_canbusdata_meas_three_sem;
extern CAN_BusDataMeas  *sm_canbusdata_meas_three_data;

extern smCANBusDataDes  *sm_canbusdata_des_four;
extern SEM_ID            sm_canbusdata_des_four_sem;
extern CAN_BusDataDes   *sm_canbusdata_des_four_data;

extern smCANBusDataMeas *sm_canbusdata_meas_four;
extern SEM_ID            sm_canbusdata_meas_four_sem;
extern CAN_BusDataMeas  *sm_canbusdata_meas_four_data;

extern int nMeasMsg;
extern int nDesMsg;

  int   init_can_shared_memory(void);
//  extern STATUS 	semGiveCount (SEM_ID semId);


#ifdef __cplusplus
}
#endif


#endif /* CANSHAREDMEMORY_HPP_ */
