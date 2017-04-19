/*!
 * @file 	DeviceEPOS2MotorParameters.hpp
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, device
 * @brief
 */

#ifndef DEVICEMOTORPARAMETERS_HPP_
#define DEVICEMOTORPARAMETERS_HPP_


//
// EPOS part
//
//Maxon EPOS Firmware specification for EPOS 70/10 EPOS24/5, 2010

#define MOTOR_TYPE_brushed_DC_motor 0x01
#define MOTOR_TYPE_EC_motor_sinus 0x0A
#define MOTOR_TYPE_EC_motor_block 0x0B

//Maxon EPOS Firmware specification
#define OPERATION_MODE_PROFILE_POSITION 0x01
#define OPERATION_MODE_PROFILE_VELOCITY 0x03
#define OPERATION_MODE_HOMING			0x06
#define OPERATION_MODE_POSITION 		-0x01
#define OPERATION_MODE_VELOCITY 		-0x02
#define OPERATION_MODE_CURRENT 			-0x03
#define OPERATION_MODE_DIAGNOSTIC 		-0x04
#define OPERATION_MODE_MASTERENCODER 	-0x05
#define OPERATION_MODE_STEPDIRECTION 	-0x06
#define OPERATION_MODE_SENSORBOARD 0x07

typedef unsigned char 	EPOS_UINT8;
typedef char 			EPOS_INT8;
typedef unsigned short 	EPOS_UINT16;
typedef short 			EPOS_INT16;
typedef unsigned int 	EPOS_UINT32;
typedef int 			EPOS_INT32;

//To convert from SI to EPOS units
#define RPM_TO_RAD_S 2.0*M_PI/60.0
#define RAD_S_TO_RPM 60/(2.0*M_PI)


//! Motor parameters for a motor controlled by a Maxon EPOS2
/*!
 * @ingroup robotCAN, device
 */
class DeviceEPOS2MotorParameters {
public:
	//! Gearratio of gearbox on motor
	double gearratio_motor;
	//! pulse number of encoder
	int encoder_pulse_number;
	//! quadrature counts = 4*encoder_pulse_number
	int encoder_qc_number;
	//! type of encoder
	int encoder_type;
	//! polarity of encoder (direction)
	int encoder_polarity;
	//! polarity of hall sensor (direction)
	int hall_polarity;
	//! type of the motor (use defines above)
	int motor_type;
	//! number of pole pair
	int pole_pair_number;
	//! thermal time constant winding [1/10 s]
	int thermal_time_constant_winding;
	//! maximum profile velocity [rad/s]
	double max_profile_velocity;
	//! continuous current limit [A]
	double continuous_current_limit;
	//! output current limit (peak current over short time)[A]
	double output_current_limit;
	//! profile acceleration [rad/s2]
	double profile_acceleration;
	//! profile decceleration [rad/s2]
	double profile_decceleration;
	//! maximal following error [rad] (maximal allowed difference of position actual value to position demand value)
	double max_following_error;

	//Conversion variables:
	double TICKS_TO_RAD;
	double RAD_TO_TICKS;
	double rad_s_Gear_to_rpm_Motor;

	//Gains
	//Dangerous: In Maxon units (for convenience)
	//! proportional gain of position controller
	int position_P_Gain; ///< 0-32767
	//! integral gain of position controller
	int position_I_Gain; ///< 0-32767
	//! derivative gain of position controller
	int position_D_Gain; ///< 0-32767

	//! proportional gain of velocity controller
	int velocity_P_Gain; ///< 0-32767
	//! integral gain of velocity controller
	int velocity_I_Gain; ///< 0-32767
	//! velocity feedforward gain of velocity controller
	int velocity_VFF_Gain;
	//! acceleration feedforward gain of velocity controller
	int velocity_AFF_Gain;

	//! proportional gain of current controller
	int current_P_Gain;
	//! integral gain of current controller
	int current_I_Gain;

	//! position limits [rad]
	double positionLimits[2];

	//! operation (control) mode (velocity, position, current, etc. Use defines above)
	int operationMode;

	//! identifier of shared memory of the Sync PDO
	int rxSYNCSMId_;

	//! identifier of shared memory of the first RxPDO
	int rxPDO1SMId_;

	//! identifier of shared memory of the second RxPDO
	int rxPDO2SMId_;

	//! identifier of shared memory of the third RxPDO
	int rxPDO3SMId_;

	//! identifier of shared memory of the fourth RxPDO
	int rxPDO4SMId_;

	//! identifier of shared memory of the first TxPDO
	int txPDO1SMId_;

	//! identifier of shared memory of the second TxPDO
	int txPDO2SMId_;

	//! identifier of shared memory of the third TxPDO
	int txPDO3SMId_;

	//! identifier of shared memory of the fourth TxPDO
	int txPDO4SMId_;

	//! identifier of shared memory of received SDO
	int inSDOSMId_;

	//! identifier of shared memory of sent SDO
	int outSDOSMId_;

	//! identifier of shared memory of received NMT
	int inNMTSMId_;

	//! identifier of shared memory of sent NMT
	int outNMTSMId_;

	/*! Motor parameters for EPOS2
	 *
	 * @param rxPDOSMId		identifier of shared memory of RxPDO
	 * @param txPDOSMId		identifier of shared memory of TxPDO
	 * @param inSDOSMId		identifier of shared memory of received SDO
	 * @param outSDOSMId	identifier of shared memory of sent SDO
	 */
	DeviceEPOS2MotorParameters(unsigned int rxSYNCSMId,
            unsigned int rxPDO1SMId,
            unsigned int rxPDO2SMId,
            unsigned int rxPDO3SMId,
            unsigned int rxPDO4SMId,
            unsigned int txPDO1SMId,
            unsigned int txPDO2SMId,
            unsigned int txPDO3SMId,
            unsigned int txPDO4SMId,
            unsigned int inSDOSMId,
            unsigned int outSDOSMId,
            unsigned int inNMTSMId,
            unsigned int outNMTSMId) :
        rxSYNCSMId_(rxSYNCSMId),
        rxPDO1SMId_(rxPDO1SMId),
        rxPDO2SMId_(rxPDO2SMId),
        rxPDO3SMId_(rxPDO3SMId),
        rxPDO4SMId_(rxPDO4SMId),
        txPDO1SMId_(txPDO1SMId),
        txPDO2SMId_(txPDO2SMId),
        txPDO3SMId_(txPDO3SMId),
        txPDO4SMId_(txPDO4SMId),
        inSDOSMId_(inSDOSMId),
        outSDOSMId_(outSDOSMId),
        inNMTSMId_(inNMTSMId),
        outNMTSMId_(outNMTSMId)
	{

	}
	virtual ~DeviceEPOS2MotorParameters()
	{

	}
};

#endif /* DEVICEMOTORPARAMETERS_HPP_ */
