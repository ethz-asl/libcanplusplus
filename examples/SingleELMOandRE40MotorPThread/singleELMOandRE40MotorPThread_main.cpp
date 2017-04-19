/*!
* @file 	singleEPOS2and4poleMotorPThread_main.cpp
* @author 	Christian Gehring
* @date		Mar 16, 2012
* @version 	0.0
* @ingroup 	libCAN
* @brief	This program controls a 4pole Maxon Motor using
* 			an EPOS 2 motor controller.
* 			It communicates by means of CAN channel 0.
* 			The libCAN is without SL and with PThread.
* 			The program runs a state machine, which
* 			initializes the motor first and turns the motor with
* 			constant speed.
*/

/* CAN Driver */
#include <linux/cpc.h>
#include <cpclib.h>

/* libCAN */
#include "CANPThread.h"

/* devices */
#include "DeviceELMOMotor.hpp"
#include "DeviceELMOMotorParametersESA.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <signal.h>


//#include <ctime>
using namespace std;

/* Set CAN bitrate to 1Mbps */
#define BTR0 0x00
#define BTR1 0x14
/* Set CAN bitrate to 500kbps */
//#define BTR0 0x00
//#define BTR1 0x1c


//! struct of bus routine initial arguments
struct BusRoutineArguments {
	// identifier of the bus [0,3]
	int iBus;
	// handle of CAN bus
	int handle;
	// time step in ms
	double time_step_ms;
};


//! RxPDO index of RxPDOManager in busManager
enum RxPDOs {
	RxPDO_SYNC = 0,
	RxPDO_MOTOR = 1,
};

//! TxPDO index of TxPDOManager in busManager
enum TxPDOs {
	TxPDO_MOTOR,
};

//! shared memory ID of sent messages
enum DESSMID {
	DESSMID_RxPDO_SYNC = 2,
	DESSMID_RxPDO_PROFILE = 0,
	DESSMID_RxPDO_ELMO_BIC = 1,
	DESSMID_SDO = 3,
};

//! shared memory ID of received messages
enum MEASSMID {
	MEASSMID_TxPDO_POSITION_VELOCITY = 0,
	MEASSMID_SDO_MOTOR = 1,
	MEASSMID_TxPDO_CURRENT = 2,
	MEASSMID_TxPDO_ANALOG = 3
};

//! CAN Node ID given by the DIP switches
enum CANNODEID {
	NODEID_MOTOR=3,
};


/* parameters */

//! number of buses
const int nBuses = 1;


//! cycle rate of loop in millisec
const double time_step_ms = 33;//2.5;

//! bus rate
int motor_servo_rate = (int) 1000.0/time_step_ms;

//! manager of all CAN buses
BusManager busManager;

static bool hasTerminated = false;

//! bus routine initial arguments
BusRoutineArguments busRoutineArgs[nBuses];

//! thread task
pthread_t bus_task;
pthread_t main_task;

enum STATEMACHINE  {
	SM_INIT_MOTOR,
	SM_INITIALIZING_MOTOR,
	SM_RUN,
	SM_EMERGENCY_STOP,
	SM_IDLE,
	SM_RUN_PROFILE_POSITION,
	SM_RUN_PROFILE_POSITION_PART2
};

//! state machine
STATEMACHINE stateSM = SM_INIT_MOTOR;


/* Prototypes */

//! thread function
void* main_routine(void *arg);
void* bus_routine(void *arg);

//! signal handler
void catch_signal(int sig);

//! emegency stop
void emergency_stop(void);

/*! message handler function that is invoked in case a CAN message arrived
 * @param 	handle			handle of the CAN bus given by CPC_OpenChannel()
 * @param	cpcmsg			CAN message from CPC driver
 * @param	customPointer 	custom pointer
 */
void msg_handler(int handle, const CPC_MSG_T * cpcmsg, void *customPointer);

/*! Maps the COB ID of a CAN message to the index of shared memory.
 * @param	iBus	number of bus
 * @param 	COBId	COB Id of message
 * @return the index of the shared memory. Returns -1 if there is no mapping
 */
int getMsgIdxFromCOBId(int iBus, int COBId);

/*! Decodes an emergency object received from an EPOS
 * @param	DATA	CAN message
 * @return	true if it is really an error
 */
bool decodeEmergencyObject(unsigned char* DATA);

/*! Sets the motor velocity
 * @param velocity velocity in rad/s
 */
void setMotorVelocity(double velocity);
void setMotorPosition(double position);

//! Prints received position and velocity of the motor
void printPositionVelocity(void);

//////////////////////////////////////////////////////////////////////////////
int main(int argc, char** args)
{
	CAN_BusDataDes canDataDes[nBuses][nDesMsg];

	/* install signal handlers */
	atexit(removeSharedMemoryAtExit);
	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);
	signal(SIGSEGV, catch_signal);

	servo_base_rate = motor_servo_rate;

	// init shared memory that is used to communicate between the main and bus routine
	if(!init_can_shared_memory()) {
		printf("Could not init shared memory!");
		exit(-1);
	}

	/* initialize the initial arguments of the bus routines */
	for (int i=0; i<nBuses; i++) {
		busRoutineArgs[i].iBus = i;
		busRoutineArgs[i].time_step_ms = time_step_ms;
		busRoutineArgs[i].handle = 0;
	}

	/* add devices to bus manager */
	for (int iBus=0; iBus<nBuses; iBus++) {
		busManager.addBus(new Bus(iBus));
		busManager.getBus(iBus)->getRxPDOManager()->addPDO(new RxPDOSync(DESSMID_RxPDO_SYNC));
		//busManager.getBus(iBus)->getRxPDOManager()->addPDO(new RxPDOELMOBinaryInterpreterCmd(DESSMID_RxPDO_ELMO_BIC));
		busManager.getBus(iBus)->getDeviceManager()->addDevice(new DeviceELMOMotor(NODEID_MOTOR, new Maxon_REmax24_Enc500(
																														DESSMID_RxPDO_PROFILE,
																														0,
																														0,
																														MEASSMID_TxPDO_ANALOG,
																														MEASSMID_TxPDO_POSITION_VELOCITY,
																														MEASSMID_TxPDO_CURRENT,
																														MEASSMID_SDO_MOTOR,
																														DESSMID_SDO)));
	}

	/* initialize desired CAN commands to zero */
	for (int iBus=0; iBus<nBuses; iBus++) {
		for (int iMsg=0; iMsg<nDesMsg; iMsg++) {
			initCANBusDataDes(&canDataDes[iBus][iMsg]);
		}
	}

	int rc = pthread_create(&main_task, NULL,  main_routine, (void *) &busRoutineArgs[0]);
    assert(0 == rc);

	rc = pthread_create(&bus_task, NULL, bus_routine, (void *) &busRoutineArgs[0]);
    assert(0 == rc);


    printf("wait for termination of bus task\n");
    rc = pthread_join(bus_task, NULL);
    assert(0 == rc);

    printf("wait for termination of main task\n");
    rc = pthread_join(main_task, NULL);
    assert(0 == rc);

}
//////////////////////////////////////////////////////////////////////////////
void* main_routine(void *arg)
{

	// variables for timing
	struct timespec   ts;
	struct timeval    start_tp;

	CAN_BusDataMeas canDataMeas[nBuses][nMeasMsg];
	CAN_BusDataDes canDataDes[nBuses][nDesMsg];


	// initialize time
	gettimeofday(&start_tp, NULL);
	ts.tv_sec = start_tp.tv_sec;
	ts.tv_nsec = start_tp.tv_usec*1000;

	int counter = 0;



	/*******************************************************
	 * LOOP
	 *******************************************************/
	while (true) {
		// increment the time
		ts.tv_nsec += (int)(1000000000./(double)motor_servo_rate);
		ts.tv_sec  += (int)(ts.tv_nsec/1000000000.);
		ts.tv_nsec  = ts.tv_nsec%1000000000;
		clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts,NULL);



		/*******************************************************
		 * READ MEASUREMENTS FROM SHARED MEMORY
		 *******************************************************/
		for (int i=0; i<nBuses; i++) {
			process_main_meas(&canDataMeas[i][0],i);
		}


		/*******************************************************
		 * READ RECEIVED CAN MESSAGES
		 *******************************************************/
		for (int iBus=0; iBus<nBuses; iBus++) {
			Bus* bus = busManager.getBus(iBus);

			/* TxPDOs */
			for (int iPDO=0; iPDO<bus->getTxPDOManager()->getSize(); iPDO++) {
				int smId = bus->getTxPDOManager()->getPDO(iPDO)->getSMId();
				bus->getTxPDOManager()->getPDO(iPDO)->setCANMsg(&canDataMeas[iBus][smId]);
			}
			/* SDO */
			SDOMsg* sdo = bus->getSDOManager()->getReceiveSDO();
			int smId = sdo->getInputMsg()->getSMId();
			if (smId != -1) {
				sdo->receiveMsg(&canDataMeas[iBus][smId]);
			}
		}

		/*******************************************************
		 * RUN TASK
		 *******************************************************/
		// do something
		switch (stateSM) {
		case SM_INIT_MOTOR:
			if (counter*time_step_ms/1000.0 > 2.0) {
				printf("initializing motor\n");
				for (int iBus=0; iBus<nBuses; iBus++) {
					DeviceELMOMotor* motor = (DeviceELMOMotor*) busManager.getBus(iBus)->getDeviceManager()->getDevice(0);
					motor->initDevice();
				}

				stateSM = SM_INITIALIZING_MOTOR;
			}
			break;
		case SM_INITIALIZING_MOTOR:

			if (counter*time_step_ms/1000.0 > 10.0) {
				for (int iBus=0; iBus<nBuses; iBus++) {
					busManager.getBus(iBus)->getRxPDOManager()->setSending(true);
//					DeviceELMOMotor* motor = (DeviceELMOMotor*) busManager.getBus(iBus)->getDeviceManager()->getDevice(0);
//					motor->executeCommandBegin();
				}
				printf("run motor\n");
				stateSM = SM_RUN;
			}
			break;
		case SM_RUN:
			setMotorVelocity(0.1);
			printPositionVelocity();
			if (counter*time_step_ms/1000.0 > 20.0) {
				printf("emergency stop\n");
				stateSM = SM_EMERGENCY_STOP;
			}
			break;
		case SM_RUN_PROFILE_POSITION:

			setMotorPosition(0.3);
			printPositionVelocity();
			if (counter*time_step_ms/1000.0 > 20.0) {
				printf("part 2\n");
				stateSM = SM_RUN_PROFILE_POSITION_PART2;
				DeviceELMOMotor* motor = (DeviceELMOMotor*) busManager.getBus(0)->getDeviceManager()->getDevice(0);
				//motor->sendControlWord();
			}
			break;
		case SM_RUN_PROFILE_POSITION_PART2:
		{
			double pos = 0.3+0.2*sin(0.1*2*M_PI*(counter*time_step_ms/1000.0));
			//printf("desired position=%lf\n", pos);
			setMotorPosition(pos);
			printPositionVelocity();
			if (counter*time_step_ms/1000.0 > 60.0) {
				printf("emergency stop\n");
				stateSM = SM_EMERGENCY_STOP;
			}
		}
			break;
		case SM_EMERGENCY_STOP:
			setMotorVelocity(0.0);
			break;
		case SM_IDLE:
		{
			DeviceELMOMotor* motor = (DeviceELMOMotor*) busManager.getBus(0)->getDeviceManager()->getDevice(0);
			bool isEnabled = false;
			if(motor->getIsMotorEnabled(isEnabled)){
				if (isEnabled) {
					printf("motor is enabled\n");
				} else {
					printf("motor is not enabled\n");
				}
			}
		}
			break;
		default:
			break;
		}


		/*******************************************************
		 * FILL CAN MESSAGES TO SEND
		 *******************************************************/
		for (int iBus=0; iBus<nBuses; iBus++) {
			for (int iMsg=0; iMsg<nDesMsg; iMsg++) {
				canDataDes[iBus][iMsg].flag = 0;
			}
		}

		for (int iBus=0; iBus<nBuses; iBus++) {
			Bus* bus = busManager.getBus(iBus);
			/* RxPDOs */
			PDOManager* pdoManager = bus->getRxPDOManager();
			if (pdoManager->isSending()) {
				for (int iPDO=0; iPDO<pdoManager->getSize(); iPDO++)
				{
					int smId = pdoManager->getPDO(iPDO)->getSMId();
					pdoManager->getPDO(iPDO)->getCANMsg(&canDataDes[iBus][smId]);
				}
			}
			/* SDO */
			SDOMsg* sdo = bus->getSDOManager()->getSendSDO();
			int smId = sdo->getOutputMsg()->getSMId();
			if (smId != -1) {
				sdo->sendMsg(&canDataDes[iBus][smId]);
			}
		}

		/*******************************************************
		 * WRITE COMMANDS TO SHARED MEMORY
		 *******************************************************/
		for (int i=0; i<nBuses; i++) {
			process_main_des(&canDataDes[i][0],i);
		}

		counter++;
	} // end loop
}


//////////////////////////////////////////////////////////////////////////////
void* bus_routine(void *arg)
{

	// variables for timing
	struct timespec   ts;
	struct timeval    start_tp;

	// name of the channel
	char channelname[6];


	//! CAN messages to send from shared memory
	CAN_BusDataDes canDataDes[nBuses][nDesMsg];
	//! if true, device is unplugged
	bool unpluggedBus[nBuses];
	//! number of not transmitted messages
	int noTransmitCounter[nBuses];
	//! maximum allowed number of not transmitted messages -> otherwise device is unplugged
	int maxNoTransmitCounter = 5;
	//! received message
	CPC_MSG_T *pMsg;

	/* get bus data */
	struct	BusRoutineArguments *busRoutineArgs = (struct BusRoutineArguments*) arg;


	/* **************************
	 * CAN DRIVER SETUP
	 ***************************/
	for (int i=0; i<nBuses; i++) {
		//! devices are  not unplugged
		unpluggedBus[i] = false;

		/* open channel */
		sprintf(channelname, "CHAN0%d", busRoutineArgs[i].iBus);

		busRoutineArgs[i].handle  = CPC_OpenChannel(channelname);
		printf("Opened channel with handle: %d\n", busRoutineArgs[i].handle);

		if(busRoutineArgs[i].handle < 0) {
			printf("CPC_OpenChannel of %s failed: %d\n", channelname, busRoutineArgs[i].handle);
			exit(-1);
		}

		/* add message handler */
		CPC_AddHandlerEx( busRoutineArgs[i].handle, msg_handler, NULL);


		// This sets up the parameters used to initialize the CAN controller
		printf("Initializing CAN-Controller ... ");

		// Parameters of the can interface
		CPC_INIT_PARAMS_T *CPCInitParamsPtr;
		CPCInitParamsPtr = CPC_GetInitParamsPtr(busRoutineArgs[i].handle);
		CPCInitParamsPtr->canparams.cc_type                      = SJA1000;
		CPCInitParamsPtr->canparams.cc_params.sja1000.btr0       = BTR0;
		CPCInitParamsPtr->canparams.cc_params.sja1000.btr1       = BTR1;
		CPCInitParamsPtr->canparams.cc_params.sja1000.outp_contr = 0xda;
		CPCInitParamsPtr->canparams.cc_params.sja1000.acc_code0  = 0xff;
		CPCInitParamsPtr->canparams.cc_params.sja1000.acc_code1  = 0xff;
		CPCInitParamsPtr->canparams.cc_params.sja1000.acc_code2  = 0xff;
		CPCInitParamsPtr->canparams.cc_params.sja1000.acc_code3  = 0xff;
		CPCInitParamsPtr->canparams.cc_params.sja1000.acc_mask0  = 0xff;
		CPCInitParamsPtr->canparams.cc_params.sja1000.acc_mask1  = 0xff;
		CPCInitParamsPtr->canparams.cc_params.sja1000.acc_mask2  = 0xff;
		CPCInitParamsPtr->canparams.cc_params.sja1000.acc_mask3  = 0xff;
		CPCInitParamsPtr->canparams.cc_params.sja1000.mode       = 0;

		// init the CAN controller
		if(CPC_CANInit(busRoutineArgs[i].handle, 0) == 0) {
			printf(" done\n");
		} else {
			printf(" ERROR (exit)\n");
			exit (1);
		}

		// switch on transmission of CAN messages from CPC to PC
		printf("Switching ON transimssion of CAN messages from CPC to PC\n");
		CPC_Control(busRoutineArgs[i].handle, CONTR_CAN_Message | CONTR_CONT_ON);



	}

	// initialize time
	gettimeofday(&start_tp, NULL);
	ts.tv_sec = start_tp.tv_sec;
	ts.tv_nsec = start_tp.tv_usec*1000;




	/*******************************************************
	 * LOOP
	 *******************************************************/
	while (true) {
		// increment the time
		ts.tv_nsec += (int)(1000000000./(double)motor_servo_rate);
		ts.tv_sec  += (int)(ts.tv_nsec/1000000000.);
		ts.tv_nsec  = ts.tv_nsec%1000000000;
		clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts,NULL);


		/*******************************************************
		 * READ SHARED MEMORY
		 *******************************************************/
		for (int i=0; i<nBuses; i++) {
			process_bus_des(&canDataDes[i][0],i);
		}


		/*******************************************************
		 * READ INCOMING CAN MESSAGES
		 *******************************************************/
		for (int iBus=0; iBus<nBuses; iBus++) {
			// timeout is in milliseconds
		//	if (CPC_WaitForEvent(handle, 1, EVENT_READ) & EVENT_READ) {
					do {
						pMsg = CPC_Handle(busRoutineArgs[iBus].handle);

						if (pMsg) {
							if (pMsg->type == CPC_MSG_T_DISCONNECTED) {
								printf("Device unplugged!\n");
								unpluggedBus[iBus] = true;
								emergency_stop();
								break;
							}
						}
					} while (pMsg);
		//	} else {
		//		printf("CPC read timeout!\n");
		//	}
		}


		/*******************************************************
		 * SEND CAN MESSAGES
		 *******************************************************/
		for (int iMsg=0; iMsg<nDesMsg; iMsg++) {
			for (int iBus=0; iBus<nBuses; iBus++) {

				/* send only if flag is true */
				if (canDataDes[iBus][iMsg].flag && !unpluggedBus[iBus]) {

					CPC_CAN_MSG_T cmsg = {0x00L,0,{0,0,0,0,0,0,0,0}};
					// put the can id of the device in cmsg
					cmsg.id = canDataDes[iBus][iMsg].COBId;
					// put the length of the message in cmsg
					cmsg.length = canDataDes[iBus][iMsg].length;
					// put the data of the message in cmsg
					for(int l=0;l<cmsg.length;l++) {
						cmsg.msg[l] = canDataDes[iBus][iMsg].value[l];
					}



					/*******************************************************
					 * SEND CAN MESSAGE
					*******************************************************/
					int ret;

	//				if (CPC_WaitForEvent(busRoutineArgs[i].handle, 1, EVENT_WRITE) & EVENT_WRITE) {
						ret =  CPC_SendMsg(busRoutineArgs[iBus].handle, 0, &cmsg);
						if (ret < 0) {
							/* an error happened */
							if (ret == CPC_ERR_CAN_NO_TRANSMIT_BUF) {
								noTransmitCounter[iBus]++;
								if (noTransmitCounter[iBus] > maxNoTransmitCounter) {
									unpluggedBus[iBus] = true;
									emergency_stop();
								}
							}
							printf("ERROR Bus%d: %s\n", busRoutineArgs[iBus].iBus,
									CPC_DecodeErrorMsg(ret));
						}
	//				} else {
	//					printf("ERROR: Transmit Timeout!\n");
	//					ret = -1;
	//				}

					usleep(120);


				}

			}
		}


	} // end while

}


//////////////////////////////////////////////////////////////////////////////
void msg_handler(int handle, const CPC_MSG_T * cpcmsg, void *customPointer)
{
	CAN_BusDataMeas canDataMeas;
	const int iBus = handle;

//	printf("Received a Message!\n");

	canDataMeas.flag = 1;
	canDataMeas.COBId = cpcmsg->msg.canmsg.id;
	canDataMeas.length = cpcmsg->msg.canmsg.length;
	for (int k=0; k<8;k++) {
		canDataMeas.value[k] = cpcmsg->msg.canmsg.msg[k];
	}

	int msgIdx = getMsgIdxFromCOBId(iBus, canDataMeas.COBId);
	if (msgIdx != -1) {
		process_bus_meas(&canDataMeas, iBus, msgIdx);
	} else {
		/* check for errors sent by EPOS */
		if (canDataMeas.COBId > 0x80 && canDataMeas.COBId < 0x100) {
			if ((canDataMeas.value[0] == 0x00) && (canDataMeas.value[1] == 0x00)) {

				/* no error */
				;
			} else {
				/* error */
				emergency_stop();
				printf("\e[0;31m(COB_ID: 0x%02X / code: 0x%02X%02X)\n", canDataMeas.COBId, canDataMeas.value[1], canDataMeas.value[0]);
				printf("==============>\n");
				printf("ERROR - emergency object found\n");
				decodeEmergencyObject(canDataMeas.value);
				printf("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
						canDataMeas.value[0],
						canDataMeas.value[1],
					    canDataMeas.value[2],
					    canDataMeas.value[3],
					    canDataMeas.value[4],
					    canDataMeas.value[5],
					    canDataMeas.value[6],
					    canDataMeas.value[7]);
				printf("<==============\n\n\e[0m");

			}
		}
		printf("Warning: Received CAN message that is not handled!\n");
		printf("\e[0;31m(COB_ID: 0x%02X / code: 0x%02X%02X)\n", canDataMeas.COBId, canDataMeas.value[1], canDataMeas.value[0]);
		printf("==============>\n");
		printf("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
				canDataMeas.value[0],
				canDataMeas.value[1],
			    canDataMeas.value[2],
			    canDataMeas.value[3],
			    canDataMeas.value[4],
			    canDataMeas.value[5],
			    canDataMeas.value[6],
			    canDataMeas.value[7]);
		printf("<==============\n\n\e[0m");
	}
}


//////////////////////////////////////////////////////////////////////////////
int getMsgIdxFromCOBId(int iBus, int COBId)
{
	const int TxPDO1Id = 0x180;
	const int TxPDO2Id = 0x280;
	const int TxPDO3Id = 0x380;
	const int TxPDO4Id = 0x480;

	const int SDOId = 0x580;
	const int NMTEnteredPreOperational = 0x700;


	switch (COBId) {
	// Motor TxPDO
	case TxPDO1Id+NODEID_MOTOR:
		printf("received PDO1\n");
		return MEASSMID_TxPDO_ANALOG;
		break;
	case TxPDO2Id+NODEID_MOTOR:
		printf("received PDO2\n");
		return MEASSMID_TxPDO_ANALOG;
		break;
	case TxPDO3Id+NODEID_MOTOR:
		return MEASSMID_TxPDO_POSITION_VELOCITY;
	case TxPDO4Id+NODEID_MOTOR:
		return MEASSMID_TxPDO_CURRENT;
	// Motor  SDO
	case SDOId+NODEID_MOTOR:
	case NMTEnteredPreOperational+NODEID_MOTOR:
		return MEASSMID_SDO_MOTOR;
	default:
		/* not handled CAN message */
		return -1;
		break;
	}
	return 0;
}


bool decodeEmergencyObject(unsigned char* DATA)
{

	if ((DATA[0] == 0x00) && (DATA[1] == 0x00)) {
		printf("No error\n");
		return false;
	} else if ((DATA[0] == 0x00) && (DATA[1] == 0x10)) {
		printf("generic error\n");
	} else if ((DATA[0]==0x10)&&(DATA[1]==0x23)) {
		printf("over current error\n");
	} else if ((DATA[0]==0x20)&&(DATA[1]==0x32)) {
		printf("under voltage error\n");
	} else if ((DATA[0]==0x10)&&(DATA[1]==0x42)) {
		printf("over temperature error\n");
	} else if ((DATA[0]==0x13)&&(DATA[1]==0x51)) {
		printf("supply voltage too low error\n");
	} else if ((DATA[0]==0x00)&&(DATA[1]==0x61)) {
		printf("internal software error\n");
	} else if ((DATA[0]==0x00)&&(DATA[1]==0x61)) {
		printf("internal software error\n");
	} else if ((DATA[0]==0x11)&&(DATA[1]==0x86)) {
		printf("Following error\n");
	} else if ((DATA[0]==0x09)&&(DATA[1]==0xFF)) {
		printf("Software position limit error\n"); // 0Xff09
	} else if ((DATA[0]==0x08)&&(DATA[1]==0xFF)) {
		printf("Hall Angle detection error\n");
	} else if ((DATA[0]==0x01)&&(DATA[1]==0xFF)) {
		printf("Hall Sensor error\n");
	} else if ((DATA[0]==0x0B)&&(DATA[1]==0xFF)) {
		printf("System overloaded\n");
	} else if ((DATA[0]==0x30)&&(DATA[1]==0x81)) {
			printf("CAN Life guard error\n");
		}
	else {
		printf("Error not specified\n");
	}
	return true;
}



//////////////////////////////////////////////////////////////////////////////
void catch_signal(int sig)
{

	// the stop procedure needs to be run only once!
	if (!hasTerminated) {
		hasTerminated = true;
		// run emergency stop
		emergency_stop();


		printf("Terminate now!\n");

		/* write emergency stop commands to shared memory */

		CAN_BusDataDes canDataDes[nBuses][nDesMsg];

		 /* FILL CAN MESSAGES TO SEND */
		for (int iBus=0; iBus<nBuses; iBus++) {
			for (int iMsg=0; iMsg<nDesMsg; iMsg++) {
				canDataDes[iBus][iMsg].flag = 0;
			}
		}

		for (int iBus=0; iBus<nBuses; iBus++) {
			Bus* bus = busManager.getBus(iBus);
			/* RxPDOs */
			PDOManager* pdoManager = bus->getRxPDOManager();
			if (pdoManager->isSending()) {
				for (int iPDO=0; iPDO<pdoManager->getSize(); iPDO++)
				{
					int smId = pdoManager->getPDO(iPDO)->getSMId();
					pdoManager->getPDO(iPDO)->getCANMsg(&canDataDes[iBus][smId]);
				}
			}
			/* SDO */
			SDOMsg* sdo = bus->getSDOManager()->getSendSDO();
			int smId = sdo->getOutputMsg()->getSMId();
			if (smId != -1) {
				sdo->sendMsg(&canDataDes[iBus][smId]);
			}
		}

		/* WRITE COMMANDS TO SHARED MEMORY */
		for (int i=0; i<nBuses; i++) {
			process_main_des(&canDataDes[i][0],i);
		}



		/* wait some time */
		struct timespec   ts;
		struct timeval    start_tp;
		const int time_step_ms = 1000;
		gettimeofday(&start_tp, NULL);
		ts.tv_sec = start_tp.tv_sec;
		ts.tv_nsec = start_tp.tv_usec*1000;
		ts.tv_nsec += (long int)(time_step_ms*1000000.0);
		ts.tv_sec  += (int)(ts.tv_nsec/1000000000.);
		ts.tv_nsec  = ts.tv_nsec%1000000000;
		if (clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts,NULL) != 0) {
			printf("ERROR\n");
		}


		// delete the real-time task
		pthread_cancel(bus_task);
		pthread_cancel(main_task);


		// close CAN channels
		for (int i=0; i<nBuses; i++) {
			CPC_RemoveHandlerEx(busRoutineArgs[i].handle, msg_handler);
			CPC_CloseChannel(busRoutineArgs[i].handle);
		}



	}
}

//////////////////////////////////////////////////////////////////////////////
void emergency_stop(void)
{
	printf("Emergency Stop\n");
	stateSM = SM_EMERGENCY_STOP;
	setMotorVelocity(0.0);

}

//////////////////////////////////////////////////////////////////////////////
void setMotorVelocity(double velocity)
{
	for (int iBus=0; iBus<nBuses; iBus++) {
		DeviceELMOMotor* motor = (DeviceELMOMotor*) busManager.getBus(iBus)->getDeviceManager()->getDevice(0);
		motor->setVelocity(velocity);
	}
}
//////////////////////////////////////////////////////////////////////////////
void setMotorPosition(double position)
{
	for (int iBus=0; iBus<nBuses; iBus++) {
		DeviceELMOMotor* motor = (DeviceELMOMotor*) busManager.getBus(iBus)->getDeviceManager()->getDevice(0);
		motor->setPosition(position);
	}
}
//////////////////////////////////////////////////////////////////////////////
void printPositionVelocity(void)
{
	double position, velocity;
	double analog, current;
	for (int iBus=0; iBus<nBuses; iBus++) {
		DeviceELMOMotor* motor = (DeviceELMOMotor*) busManager.getBus(iBus)->getDeviceManager()->getDevice(0);
		position = motor->getPosition();
		velocity = motor->getVelocity();
		printf("Bus%d: position=%f [rad]\tvelocity=%f [rad/s]\n", iBus, position, velocity);
		analog = motor->getAnalog();
		current = motor->getCurrent();
		printf("Bus%d: analog=%f \tcurrent=%f\n", iBus, analog, current);
//		double value;
//		if (motor->getAnalogInputOne(value)) {
//			printf("Bus%d: analog=%f\n",iBus, value);
//		}
	}
}
