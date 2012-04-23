/*!
 * @file 	CANSharedMemoryFct.hpp
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 */


#ifndef CANSHAREDMEMORYFCT_HPP_
#define CANSHAREDMEMORYFCT_HPP_

#include "LibCANConfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/* local functions */
int process_bus_meas(CAN_BusDataMeas *canDataMeas, int iBus, int iMsg);
int process_bus_des(CAN_BusDataDes *canDataDes, int iBus);
int process_main_meas(CAN_BusDataMeas *canDataMeas, int iBus);
int process_main_des(CAN_BusDataDes *canDataDes, int iBus);


int take_sem_des_ready(int iBus);
int give_sem_des_ready(int iBus);

void copyBusDataDes(CAN_BusDataDes* canDataDesSrc, CAN_BusDataDes* canDataDesTarget);
void copyBusDataMeas(CAN_BusDataMeas* canDataMeasSrc, CAN_BusDataMeas* canDataMeasTarget);

void initCANBusDataMeas(CAN_BusDataMeas* canDataMeas);
void initCANBusDataDes(CAN_BusDataDes* canDataDes);

#ifdef __cplusplus
}
#endif

#endif /* CANSHAREDMEMORYFCT_HPP_ */
