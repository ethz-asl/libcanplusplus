/*!
 * @file 	CANSharedMemoryFct.c
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 */

#include "CANSharedMemoryDecl.hpp"
#include "LibCANConfig.h"
#ifdef USE_SL_FUNCTIONS
#include "SL_vx_wrappers.h"
#include "utility.h"
#else
	#include "SLVxWrapper.hpp"
	#include "SLUtility.h"
#endif
#include "CANSharedMemory.hpp"
#include "CANSharedMemoryFct.hpp"


#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define TIME_OUT_NS  500000 //[0.5ms] //1000000000 [1s]
#define TIME_OUT_BUS_NS 50000 // [50 us]

//////////////////////////////////////////////////////////////////////////////
int process_bus_meas(CAN_BusDataMeas *canDataMeas, int iBus, int iMsg)
{
	SEM_ID sm_sem;
	CAN_BusDataMeas* sm_data;

	switch (iBus) {
	case 0:
		sm_sem = sm_canbusdata_meas_one_sem;
		sm_data = sm_canbusdata_meas_one->busData;
		break;
	case 1:
		sm_sem = sm_canbusdata_meas_two_sem;
		sm_data = sm_canbusdata_meas_two->busData;
		break;
	case 2:
		sm_sem = sm_canbusdata_meas_three_sem;
		sm_data = sm_canbusdata_meas_three->busData;
		break;
	case 3:
		sm_sem = sm_canbusdata_meas_four_sem;
		sm_data = sm_canbusdata_meas_four->busData;
		break;
	default:
		printf("Error: process_bus_meas: iBus is wrong!\n");
		exit(-1);
	}


	if (semTake(sm_sem,ns2ticks(TIME_OUT_BUS_NS)) == ERROR) {
		printf("Warning: process_bus_meas: Could not take sem!\n");
	   // no receive
	} else {
		/* copy data */
		copyBusDataMeas(canDataMeas, &sm_data[iMsg]);
		semGive(sm_sem);
	}


    return TRUE;
}
int process_bus_des(CAN_BusDataDes *canDataDes, int iBus)
{
	int i;
	SEM_ID sm_sem;
	CAN_BusDataDes* sm_data;

	switch (iBus) {
	case 0:
		sm_sem = sm_canbusdata_des_one_sem;
		sm_data = sm_canbusdata_des_one->busData;
		break;
	case 1:
		sm_sem = sm_canbusdata_des_two_sem;
		sm_data = sm_canbusdata_des_two->busData;
		break;
	case 2:
		sm_sem = sm_canbusdata_des_three_sem;
		sm_data = sm_canbusdata_des_three->busData;
		break;
	case 3:
		sm_sem = sm_canbusdata_des_four_sem;
		sm_data = sm_canbusdata_des_four->busData;
		break;
	default:
		printf("Error: iBus is wrong!\n");
		exit(-1);
	}

// ns2ticks(TIME_OUT_BUS_NS) NO_WAIT
	if (semTake(sm_sem,ns2ticks(TIME_OUT_BUS_NS)) == ERROR) {
		  printf("Warning: process_bus_des: Could not take sem!\n");
		  // no receive
	} else {

		/* copy data */
		for (i=0; i<nDesMsg; i++){
			copyBusDataDes(&sm_data[i], &canDataDes[i]);
		//				initCANBusDataDes(&sm_data[i]);
		}
		semGive(sm_sem);
	}

    return TRUE;
}

int take_sem_des_ready(int iBus)
{
	int  wait_flag;


		wait_flag = WAIT_FOREVER; //WAIT_FOREVER; //NO_WAIT

		SEM_ID sm_ready_sem;




		switch (iBus) {
		case 0:
			sm_ready_sem = sm_canbusdata_des_one_ready_sem;
			break;
		case 1:
			sm_ready_sem = sm_canbusdata_des_two_ready_sem;
			break;
		case 2:
			sm_ready_sem = sm_canbusdata_des_three_ready_sem;
			break;
		case 3:
			sm_ready_sem = sm_canbusdata_des_four_ready_sem;
			break;
		default:
			printf("Error: iBus is wrong!\n");
			exit(-1);
		}

		if (semTake(sm_ready_sem,wait_flag) == ERROR) {
			  printf("des_ready_sem\n");
			if (wait_flag == WAIT_FOREVER) // an error in WAIT_FOREVER must be terminated
				exit(-1);

			//count_no_receive_total += 1./(double)task_servo_ratio;
		} else {
			;
		}
		return 0;
}

int give_sem_des_ready(int iBus)
{
	SEM_ID sm_ready_sem;

	switch (iBus) {
	case 0:
		sm_ready_sem = sm_canbusdata_des_one_ready_sem;
		break;
	case 1:
		sm_ready_sem = sm_canbusdata_des_two_ready_sem;
		break;
	case 2:
		sm_ready_sem = sm_canbusdata_des_three_ready_sem;
		break;
	case 3:
		sm_ready_sem = sm_canbusdata_des_four_ready_sem;
		break;
	default:
		printf("Error: iBus is wrong!\n");
		exit(-1);
	}

	semGive(sm_ready_sem);
	return 0;
}

//////////////////////////////////////////////////////////////////////////////
int process_main_meas(CAN_BusDataMeas *canDataMeas, int iBus)
{
    SEM_ID sm_sem;
    CAN_BusDataMeas* sm_data;
    int i;

	switch (iBus) {
	case 0:
		sm_sem = sm_canbusdata_meas_one_sem;
		sm_data = sm_canbusdata_meas_one->busData; //sm_canbusdata_meas_one_data;
		break;
	case 1:
		sm_sem = sm_canbusdata_meas_two_sem;
		sm_data = sm_canbusdata_meas_two->busData; //sm_canbusdata_meas_two_data;
		break;
	case 2:
		sm_sem = sm_canbusdata_meas_three_sem;
		sm_data = sm_canbusdata_meas_three->busData;
		break;
	case 3:
		sm_sem = sm_canbusdata_meas_four_sem;
		sm_data = sm_canbusdata_meas_four->busData;
		break;
	default:
		printf("Error: iBus is wrong!\n");
		exit(-1);
	}

// ns2ticks(TIME_OUT_NS) NO_WAIT
	if (semTake(sm_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
		printf("Warning: process_main_meas: Could not take sem!\n");
	} else {

		for (i=0; i<nMeasMsg; i++){
			copyBusDataMeas(&sm_data[i],&canDataMeas[i]);
			initCANBusDataMeas(&sm_data[i]);
		}

		semGive(sm_sem);
	}

	return TRUE;
}

//////////////////////////////////////////////////////////////////////////////
int process_main_des(CAN_BusDataDes *canDataDes, int iBus)
{

    SEM_ID sm_sem;
    CAN_BusDataDes* sm_data;
    int i;

	switch (iBus) {
	case 0:
		sm_sem = sm_canbusdata_des_one_sem;
		sm_data = sm_canbusdata_des_one->busData; //sm_canbusdata_des_one_data;
		break;
	case 1:
		sm_sem = sm_canbusdata_des_two_sem;
		sm_data = sm_canbusdata_des_two->busData; //sm_canbusdata_des_two_data;
		break;
	case 2:
		sm_sem = sm_canbusdata_des_three_sem;
		sm_data = sm_canbusdata_des_three->busData;
		break;
	case 3:
		sm_sem = sm_canbusdata_des_four_sem;
		sm_data = sm_canbusdata_des_four->busData;
		break;
	default:
		printf("Error: iBus is wrong!\n");
		exit(-1);
	}

//	ns2ticks(TIME_OUT_NS) NO_WAIT
	if (semTake(sm_sem,ns2ticks(TIME_OUT_NS)) == ERROR) {
		printf("Warning: process_main_des: Could not take sem!\n");
	} else {
		for (i=0; i<nDesMsg; i++){
			copyBusDataDes(&canDataDes[i], &sm_data[i]);
		}
		semGive(sm_sem);
	}

	return TRUE;
}

//////////////////////////////////////////////////////////////////////////////
void copyBusDataDes(CAN_BusDataDes* canDataDesSrc, CAN_BusDataDes* canDataDesTarget)
{
	int k;
	canDataDesTarget->flag = canDataDesSrc->flag;
	canDataDesTarget->COBId = canDataDesSrc->COBId;
	canDataDesTarget->length = canDataDesSrc->length;
	for (k=0; k<8; k++)  {
		canDataDesTarget->value[k] = canDataDesSrc->value[k];

	}

}

//////////////////////////////////////////////////////////////////////////////
void copyBusDataMeas(CAN_BusDataMeas* canDataMeasSrc, CAN_BusDataMeas* canDataMeasTarget)
{
	int k;
	canDataMeasTarget->flag = canDataMeasSrc->flag;
	canDataMeasTarget->COBId = canDataMeasSrc->COBId;
	canDataMeasTarget->length = canDataMeasSrc->length;
	for (k=0; k<8; k++)  {
		canDataMeasTarget->value[k] = canDataMeasSrc->value[k];
	}

}

//////////////////////////////////////////////////////////////////////////////
void initCANBusDataMeas(CAN_BusDataMeas* canDataMeas)
{
	int k;
	canDataMeas->flag = 0;
	canDataMeas->COBId = 0;
	canDataMeas->length = 0;
	for (k=0; k<8; k++)  {
		canDataMeas->value[k] = 0;
	}

}
//////////////////////////////////////////////////////////////////////////////
void initCANBusDataDes(CAN_BusDataDes* canDataDes)
{
	int k;
	canDataDes->flag = 0;
	canDataDes->COBId = 0;
	canDataDes->length = 0;
	for (k=0; k<8; k++)  {
		canDataDes->value[k] = 0;
	}
}




#ifdef __cplusplus
}
#endif
