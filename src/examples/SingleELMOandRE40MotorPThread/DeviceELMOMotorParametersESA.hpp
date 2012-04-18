/*!
 * @file 	DeviceEPOS2MotorParametersStarleth.hpp
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 */


#ifndef DEVICEMOTORPARAMETERSSTARLETH_HPP_
#define DEVICEMOTORPARAMETERSSTARLETH_HPP_


#include "DeviceELMOMotorParameters.hpp"
#include <math.h>



struct Maxon_RE40_Enc500 : DeviceELMOMotorParameters {
	Maxon_RE40_Enc500(int rxPDO1SMId,
	  int rxPDO2SMId,
	  int txPDO1SMId,
	  int txPDO2SMId,
	  int txPDO3SMId,
	  int txPDO4SMId,
	  int inSDOSMId,
	  int outSDOSMId):
	  DeviceELMOMotorParameters(rxPDO1SMId,
								  rxPDO2SMId,
								  txPDO1SMId,
								  txPDO2SMId,
								  txPDO3SMId,
								  txPDO4SMId,
								  inSDOSMId,
								  outSDOSMId)
	{
		gearratio_motor = 101.0;		// ok
		encoder_pulse_number = 500;		// ok
		encoder_qc_number =		4.0*encoder_pulse_number; // ok

		// TODO: delete
		encoder_type = 1;
		// TODO: delete
		encoder_polarity = 0;
		// TODO: delete
		hall_polarity = 0;


		motor_type  = MOTOR_TYPE_brushed_DC_motor; // ok
		pole_pair_number  = 1;			// ok
		thermal_time_constant_winding = 416; //[1/10 s]		// ok

		//Default control mode
		operationMode = OPERATION_MODE_PROFILE_POSITION; //OPERATION_MODE_PROFILE_VELOCITY; //OPERATION_MODE_PROFILE_POSITION;

		/* high gains */
		 velocity_P_Gain = 2915;
		 velocity_I_Gain = 2222;
		 velocity_VFF_Gain = 0;
		 velocity_AFF_Gain = 0;
		 current_P_Gain  = 688;
		 current_I_Gain  = 123;

		positionLimits[0] = 0;
		positionLimits[1] = M_PI;

		continuous_current_limit = 4.7;	//[A]
		output_current_limit	  = 9.4;//[A]

		max_profile_velocity =    M_PI/2.0;		//[rad/s] of Gear output
		profile_acceleration	= 32*M_PI;			//[rad/s2]
		profile_decceleration	= 32*M_PI;			//[rad/s2]
		max_following_error 	= 0.2;						//[rad]

		TICKS_TO_RAD = (2*M_PI)/(double)encoder_qc_number ;				// ok
		RAD_TO_TICKS =1.0/TICKS_TO_RAD;									// ok
		rad_s_Gear_to_rpm_Motor =  gearratio_motor * RAD_S_TO_RPM;		// ok
		rad_s_Gear_to_counts_s_Motor = gearratio_motor * RAD_TO_TICKS;  // ok


	}
};

struct Maxon_REmax24_Enc500 : DeviceELMOMotorParameters {
	Maxon_REmax24_Enc500(int rxPDO1SMId,
	  int rxPDO2SMId,
	  int txPDO1SMId,
	  int txPDO2SMId,
	  int txPDO3SMId,
	  int txPDO4SMId,
	  int inSDOSMId,
	  int outSDOSMId):
	  DeviceELMOMotorParameters(  rxPDO1SMId,
								  rxPDO2SMId,
								  txPDO1SMId,
								  txPDO2SMId,
								  txPDO3SMId,
								  txPDO4SMId,
								  inSDOSMId,
								  outSDOSMId)
	{
		gearratio_motor = 2772.0;		// ok
		encoder_pulse_number = 512;		// ok
		encoder_qc_number =	4.0*encoder_pulse_number; // ok

		// TODO: delete
		encoder_type = 1;
		// TODO: delete
		encoder_polarity = 0;
		// TODO: delete
		hall_polarity = 0;


		motor_type  = MOTOR_TYPE_brushed_DC_motor; // ok
		pole_pair_number  = 1;			// ok
		thermal_time_constant_winding = 82; //[1/10 s]		// ok

		//Default control mode
		operationMode = OPERATION_MODE_PROFILE_POSITION; //OPERATION_MODE_PROFILE_VELOCITY; //OPERATION_MODE_PROFILE_POSITION;

		/* high gains */
		 velocity_P_Gain = 2915;
		 velocity_I_Gain = 2222;
		 velocity_VFF_Gain = 0;
		 velocity_AFF_Gain = 0;
		 current_P_Gain  = 688;
		 current_I_Gain  = 123;

		positionLimits[0] = -M_PI/2;
		positionLimits[1] = M_PI/2;

		continuous_current_limit = 0.516;	//[A] ok
		output_current_limit	  = 1.1;//[A]

		max_profile_velocity =    M_PI/2.0;		//[rad/s] of Gear output
		profile_acceleration	= 32*M_PI;			//[rad/s2]
		profile_decceleration	= 32*M_PI;			//[rad/s2]
		max_following_error 	= 0.5;						//[rad]

		TICKS_TO_RAD = (2*M_PI)/(double)encoder_qc_number ;  			// ok
		RAD_TO_TICKS =1.0/TICKS_TO_RAD;  								// ok
		rad_s_Gear_to_rpm_Motor =  gearratio_motor * RAD_S_TO_RPM;  	// ok
		rad_s_Gear_to_counts_s_Motor = gearratio_motor * RAD_TO_TICKS;  // ok

	}
};




#endif /* DEVICEMOTORPARAMETERSSTARLETH_HPP_ */
