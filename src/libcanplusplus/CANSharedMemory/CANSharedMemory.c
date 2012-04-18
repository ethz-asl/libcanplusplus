/*!
 * @file 	CANSharedMemory.c
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 */
#include "LibCANConfig.h"

#ifdef USE_SL_FUNCTIONS
#include "SL_vx_wrappers.h"
#include "utility.h"
#else
	#include "SLVxWrapper.hpp"
	#include "SLUtility.h"
#endif


#include "CANSharedMemory.hpp"

#ifdef USE_SL_FUNCTIONS
	#include "SL.h"
	#include "SL_user.h"
	#include "SL_shared_memory.h"
#endif
#include "SLSharedMemory.hpp"




/* global variables */
SEM_ID             sm_canbusdata_des_one_ready_sem;
SEM_ID             sm_canbusdata_des_two_ready_sem;
SEM_ID             sm_canbusdata_des_three_ready_sem;
SEM_ID             sm_canbusdata_des_four_ready_sem;

SEM_ID             sm_canbusdata_meas_one_ready_sem;
SEM_ID             sm_canbusdata_meas_two_ready_sem;
SEM_ID             sm_canbusdata_meas_three_ready_sem;
SEM_ID             sm_canbusdata_meas_four_ready_sem;


SEM_ID             sm_canbus_init_one_sem;
SEM_ID             sm_canbus_init_two_sem;
SEM_ID             sm_canbus_init_three_sem;
SEM_ID             sm_canbus_init_four_sem;

SEM_ID             sm_canbus_wait_one_sem;
SEM_ID             sm_canbus_wait_two_sem;
SEM_ID             sm_canbus_wait_three_sem;
SEM_ID             sm_canbus_wait_four_sem;

smCANBusDataDes  *sm_canbusdata_des_one;
SEM_ID            sm_canbusdata_des_one_sem;
CAN_BusDataDes   *sm_canbusdata_des_one_data;

smCANBusDataMeas *sm_canbusdata_meas_one;
SEM_ID            sm_canbusdata_meas_one_sem;
CAN_BusDataMeas  *sm_canbusdata_meas_one_data;

smCANBusDataDes  *sm_canbusdata_des_two;
SEM_ID            sm_canbusdata_des_two_sem;
CAN_BusDataDes   *sm_canbusdata_des_two_data;

smCANBusDataMeas *sm_canbusdata_meas_two;
SEM_ID            sm_canbusdata_meas_two_sem;
CAN_BusDataMeas  *sm_canbusdata_meas_two_data;

smCANBusDataDes  *sm_canbusdata_des_three;
SEM_ID            sm_canbusdata_des_three_sem;
CAN_BusDataDes   *sm_canbusdata_des_three_data;

smCANBusDataMeas *sm_canbusdata_meas_three;
SEM_ID            sm_canbusdata_meas_three_sem;
CAN_BusDataMeas  *sm_canbusdata_meas_three_data;

smCANBusDataDes  *sm_canbusdata_des_four;
SEM_ID            sm_canbusdata_des_four_sem;
CAN_BusDataDes   *sm_canbusdata_des_four_data;

smCANBusDataMeas *sm_canbusdata_meas_four;
SEM_ID            sm_canbusdata_meas_four_sem;
CAN_BusDataMeas  *sm_canbusdata_meas_four_data;



#ifdef __cplusplus
extern "C" {
#endif

//! number of measurement messages per Bus (6xPDOPositionVelocity + 6xSDO)
const int nMeasMsg = 30;
//! number of desired messages per Bus (3xPDOVelocity + 1xPDOSync + 1xSDO)
const int nDesMsg = 21;


int
init_can_shared_memory(void)
{

  /********************************************************************/
  /********************************************************************/
  /* shared memory objects */
  /********************************************************************/

  /********************************************************************/
  if (init_sm_object_can("smCANDesOne",
		     sizeof(smCANBusDataDes),
		     sizeof(CAN_BusDataDes)*(nDesMsg),
		     &sm_canbusdata_des_one_sem,
		     (void **)&sm_canbusdata_des_one)) {
    ;
  } else {
    return FALSE;
  }
  sm_canbusdata_des_one_data =
    (CAN_BusDataDes *)my_calloc(nDesMsg,sizeof(CAN_BusDataDes),MY_STOP);
  /********************************************************************/
  if (init_sm_object_can("smCANMeasOne",
		     sizeof(smCANBusDataMeas),
		     sizeof(CAN_BusDataMeas)*(nMeasMsg),
		     &sm_canbusdata_meas_one_sem,
		     (void **)&sm_canbusdata_meas_one)) {
    ;
  } else {
    return FALSE;
  }
  sm_canbusdata_meas_one_data =
    (CAN_BusDataMeas *)my_calloc(nMeasMsg,sizeof(CAN_BusDataMeas),MY_STOP);

  /********************************************************************/
    if (init_sm_object_can("smCANDesTwo",
  		     sizeof(smCANBusDataDes),
  		     sizeof(CAN_BusDataDes)*(nDesMsg),
  		     &sm_canbusdata_des_two_sem,
  		     (void **)&sm_canbusdata_des_two)) {
      ;
    } else {
      return FALSE;
    }
    sm_canbusdata_des_two_data =
      (CAN_BusDataDes *)my_calloc(nDesMsg,sizeof(CAN_BusDataDes),MY_STOP);
    /********************************************************************/
    if (init_sm_object_can("smCANMT",
  		     sizeof(smCANBusDataMeas),
  		     sizeof(CAN_BusDataMeas)*(nMeasMsg),
  		     &sm_canbusdata_meas_two_sem,
  		     (void **)&sm_canbusdata_meas_two)) {
      ;
    } else {
      return FALSE;
    }
    sm_canbusdata_meas_two_data =
      (CAN_BusDataMeas *)my_calloc(nMeasMsg,sizeof(CAN_BusDataMeas),MY_STOP);
    /********************************************************************/
      if (init_sm_object_can("smCANDesThree",
    		     sizeof(smCANBusDataDes),
    		     sizeof(CAN_BusDataDes)*(nDesMsg),
    		     &sm_canbusdata_des_three_sem,
    		     (void **)&sm_canbusdata_des_three)) {
        ;
      } else {
        return FALSE;
      }
      sm_canbusdata_des_three_data =
        (CAN_BusDataDes *)my_calloc(nDesMsg,sizeof(CAN_BusDataDes),MY_STOP);
      /********************************************************************/
      if (init_sm_object_can("smCANMeasThree",
    		     sizeof(smCANBusDataMeas),
    		     sizeof(CAN_BusDataMeas)*(nMeasMsg),
    		     &sm_canbusdata_meas_three_sem,
    		     (void **)&sm_canbusdata_meas_three)) {
        ;
      } else {
        return FALSE;
      }
      sm_canbusdata_meas_three_data =
        (CAN_BusDataMeas *)my_calloc(nMeasMsg,sizeof(CAN_BusDataMeas),MY_STOP);
      /********************************************************************/
        if (init_sm_object_can("smCANDesFour",
      		     sizeof(smCANBusDataDes),
      		     sizeof(CAN_BusDataDes)*(nDesMsg),
      		     &sm_canbusdata_des_four_sem,
      		     (void **)&sm_canbusdata_des_four)) {
          ;
        } else {
          return FALSE;
        }
        sm_canbusdata_des_four_data =
          (CAN_BusDataDes *)my_calloc(nDesMsg,sizeof(CAN_BusDataDes),MY_STOP);
        /********************************************************************/
        if (init_sm_object_can("smCANMeasFour",
      		     sizeof(smCANBusDataMeas),
      		     sizeof(CAN_BusDataMeas)*(nMeasMsg),
      		     &sm_canbusdata_meas_four_sem,
      		     (void **)&sm_canbusdata_meas_four)) {
          ;
        } else {
          return FALSE;
        }
        sm_canbusdata_meas_four_data =
          (CAN_BusDataMeas *)my_calloc(nMeasMsg,sizeof(CAN_BusDataMeas),MY_STOP);
        /********************************************************************/
#ifdef USE_XSENS
        if (init_sm_object_can("smXSENSMeas",
      		     sizeof(smXSENSDataMeas),
      		     sizeof(XSENS_DataMeas)*(1),
      		     &sm_xsens_meas_sem,
      		     (void **)&sm_xsens_meas)) {
          ;
        } else {
          return FALSE;
        }
        sm_xsens_meas_data =
          (XSENS_DataMeas *)my_calloc(1,sizeof(XSENS_DataMeas),MY_STOP);
#endif

  /********************************************************************/
  /********************************************************************/
  /* shared semaphores */

  if (!init_sm_sem_can("smCANBusDataDesOneReadySem", SEM_EMPTY,(void**)&sm_canbusdata_des_one_ready_sem))
    return FALSE;
  if (!init_sm_sem_can("smCANBusDataDesTwoReadySem", SEM_EMPTY,(void**)&sm_canbusdata_des_two_ready_sem))
    return FALSE;
  if (!init_sm_sem_can("smCANBusDataDesThreeReadySem", SEM_EMPTY,(void**)&sm_canbusdata_des_three_ready_sem))
    return FALSE;
  if (!init_sm_sem_can("smCANBusDataDesFourReadySem", SEM_EMPTY,(void**)&sm_canbusdata_des_four_ready_sem))
    return FALSE;

  if (!init_sm_sem_can("smCANBusDataMeasOneReadySem", SEM_EMPTY,(void**)&sm_canbusdata_meas_one_ready_sem))
    return FALSE;
  if (!init_sm_sem_can("smCANBusDataMeasTwoReadySem", SEM_EMPTY,(void**)&sm_canbusdata_meas_two_ready_sem))
    return FALSE;
  if (!init_sm_sem_can("smCANBusDataMeasThreeReadySem", SEM_EMPTY,(void**)&sm_canbusdata_meas_three_ready_sem))
    return FALSE;
  if (!init_sm_sem_can("smCANBusDataMeasFourReadySem", SEM_EMPTY,(void**)&sm_canbusdata_meas_four_ready_sem))
    return FALSE;

#ifdef USE_XSENS
  if (!init_sm_sem_can("smXSENSDataMeasReadySem", SEM_EMPTY,(void**)&sm_xsens_meas_ready_sem))
    return FALSE;
#endif

  if (!init_sm_sem_can("smCANBusInitOneSem", SEM_EMPTY,(void**)&sm_canbus_init_one_sem))
    return FALSE;
  if (!init_sm_sem_can("smCANBusInitTwoSem", SEM_EMPTY,(void**)&sm_canbus_init_two_sem))
     return FALSE;
  if (!init_sm_sem_can("smCANBusInitThreeSem", SEM_EMPTY,(void**)&sm_canbus_init_three_sem))
     return FALSE;
  if (!init_sm_sem_can("smCANBusInitFourSem", SEM_EMPTY,(void**)&sm_canbus_init_four_sem))
     return FALSE;

  if (!init_sm_sem_can("smCANBusWaitOneSem", SEM_EMPTY,(void**)&sm_canbus_wait_one_sem))
     return FALSE;
  if (!init_sm_sem_can("smCANBusWaitTwoSem", SEM_EMPTY,(void**)&sm_canbus_wait_two_sem))
     return FALSE;
  if (!init_sm_sem_can("smCANBusWaitThreeSem", SEM_EMPTY,(void**)&sm_canbus_wait_three_sem))
     return FALSE;
  if (!init_sm_sem_can("smCANBusWaitFourSem", SEM_EMPTY,(void**)&sm_canbus_wait_four_sem))
     return FALSE;



  return TRUE;

}




#ifdef __cplusplus
}
#endif
