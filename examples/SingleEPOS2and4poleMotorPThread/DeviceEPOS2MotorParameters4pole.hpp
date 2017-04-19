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


#include "DeviceEPOS2MotorParameters.hpp"
#include <math.h>



struct Maxon_EC_4pole_200_Enc_500 : DeviceEPOS2MotorParameters {
	Maxon_EC_4pole_200_Enc_500(int rxPDOSMId, int txPDOSMId, int inSDOSMId, int outSDOSMId):
		DeviceEPOS2MotorParameters(rxPDOSMId, txPDOSMId, inSDOSMId, outSDOSMId){
		gearratio_motor = 1.0;
		encoder_pulse_number = 500;		// CG ok
		encoder_qc_number =		4.0*encoder_pulse_number;
		encoder_type = 1;
		encoder_polarity = 0;
		hall_polarity = 0;
		motor_type  = MOTOR_TYPE_EC_motor_sinus; // CG ok
		pole_pair_number  = 2;
		thermal_time_constant_winding = 21; //8; //[1/10 s]

		//Default control mode
		operationMode = OPERATION_MODE_VELOCITY;

		/* high gains */
		 velocity_P_Gain = 2915; ///< 0-32767  //default 1000  // CG ok
		 velocity_I_Gain = 924;  ///< 0-32767 //default 100	   // CG ok
		 velocity_VFF_Gain = 0;
		 velocity_AFF_Gain = 0;
		 current_P_Gain  = 688;	///< 0-32767 // default 800	   // CG ok
		 current_I_Gain  = 123;	///< 0-32767 // default 200	   // CG ok

//		 /* low gains */
//		 velocity_P_Gain = 1958;
//		 velocity_I_Gain = 422;
//		 velocity_VFF_Gain = 0.0;//4038;
//		 velocity_AFF_Gain = 0.0; //220;
//		 current_P_Gain  = 204;
//		 current_I_Gain  = 37;


		continuous_current_limit = 4.7;	//[A]
		output_current_limit	  = 9.4;//[A]

		max_profile_velocity =    M_PI/2.0;		//[rad/s] of Gear output
		profile_acceleration	= 32*M_PI;			//[rad/s2]
		profile_decceleration	= 32*M_PI;			//[rad/s2]
		max_following_error 	= 0.2;						//[rad]

		TICKS_TO_RAD = (2*M_PI)/(double)encoder_qc_number ;
		RAD_TO_TICKS =1.0/TICKS_TO_RAD;
		rad_s_Gear_to_rpm_Motor =  gearratio_motor * RAD_S_TO_RPM;


	}
};



struct Motor4poleParams : Maxon_EC_4pole_200_Enc_500 {
	Motor4poleParams(int rxPDOSMId, int txPDOSMId, int inSDOSMId, int outSDOSMId):
		Maxon_EC_4pole_200_Enc_500(rxPDOSMId, txPDOSMId, inSDOSMId, outSDOSMId)
	{
		gearratio_motor = 1.0;
		encoder_pulse_number = 500;		// CG ok
		encoder_qc_number =		4.0*encoder_pulse_number;
		encoder_polarity = 0;

		//Limits: Dangerous: Assuming that robot home position corresponds to maxon zero position
		positionLimits[0] =  -300.0; //limits in [rad]
		positionLimits[1] =  300.0; //For the Knee with gear 2:1

		//Overwrite generic RE_20 settings for the knee joint:
		//max_profile_velocity = 1.0;   //M_PI;		//[rad/s] of Gear output


		TICKS_TO_RAD = (2*M_PI)/(double)encoder_qc_number ;
		RAD_TO_TICKS =1.0/TICKS_TO_RAD;
		rad_s_Gear_to_rpm_Motor =  gearratio_motor * RAD_S_TO_RPM;

	}
};

#endif /* DEVICEMOTORPARAMETERSSTARLETH_HPP_ */
