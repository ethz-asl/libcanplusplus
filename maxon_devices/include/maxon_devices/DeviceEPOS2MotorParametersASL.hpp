/*!
 * @file 	DeviceEPOS2MotorParametersASL.hpp
 * @author 	Christian Gehring
 * @date 	Aug, 2012
 * @version 1.0
 * @ingroup robotCAN, device
 * @brief	Some example parameters we use at ASL
 */
#ifndef DEVICEMOTORPARAMETERSASL_HPP_
#define DEVICEMOTORPARAMETERSASL_HPP_


#include "maxon_devices/DeviceEPOS2MotorParameters.hpp"
#include <math.h>


//! Motor used in Pegasus Project as hip motor
/*! Maxon EC 4pole 30
 * Voltage: 48V
 */
struct PegasusHipMotor : DeviceEPOS2MotorParameters {
	PegasusHipMotor(int rxPDO1SMId, int rxPDO2SMId, int txPDO1SMId, int inSDOSMId, int outSDOSMId):
	DeviceEPOS2MotorParameters(rxPDO1SMId,
	  rxPDO2SMId,
	  0, // TODO: NaN, Null?
	  0,
	  txPDO1SMId,
	  0,
	  0,
	  0,
	  inSDOSMId,
	  outSDOSMId)
	{
	gearratio_motor = 28.0;
	encoder_pulse_number = 1000;
	encoder_qc_number = 4.0 * encoder_pulse_number;
	encoder_type = 1;
	encoder_polarity = 0;
	hall_polarity = 0;
	motor_type  = MOTOR_TYPE_EC_motor_sinus;
	pole_pair_number  = 2;
	thermal_time_constant_winding = 7; //[1/10 s]

	//Default control mode
	operationMode = OPERATION_MODE_VELOCITY;

	/* high gains */
	// velocity_P_Gain = 2915; ///< 0-32767  //default 1000
	// velocity_I_Gain = 924;  ///< 0-32767 //default 100
	// velocity_VFF_Gain = 0;
	// velocity_AFF_Gain = 0;
	// current_P_Gain  = 688; ///< 0-32767 // default 800
	// current_I_Gain  = 123; ///< 0-32767 // default 200

	velocity_P_Gain = 1000; ///< 0-32767  //default 1000
	velocity_I_Gain = 100;  ///< 0-32767 //default 100
	velocity_VFF_Gain = 0;
	velocity_AFF_Gain = 0;
	current_P_Gain  = 800; ///< 0-32767 // default 800
	current_I_Gain  = 200; ///< 0-32767 // default 200

	continuous_current_limit = 4.7; //[A]
	output_current_limit = 9.4; //[A]

	max_profile_velocity = M_PI/2.0; //[rad/s] of Gear output
	profile_acceleration = 32*M_PI; //[rad/s2]
	profile_decceleration = 32*M_PI; //[rad/s2]
	max_following_error = 0.2; //[rad]

	//Limits: Dangerous: Assuming that robot home position corresponds to maxon zero position
	positionLimits[0] =  -300.0; //limits in [rad]
	positionLimits[1] =  300.0; //For the Knee with gear 2:1

	TICKS_TO_RAD = (2*M_PI)/(double)encoder_qc_number ;
	RAD_TO_TICKS =1.0/TICKS_TO_RAD;
	rad_s_Gear_to_rpm_Motor =  gearratio_motor * RAD_S_TO_RPM;
	}
};



//! Motor used in Pegasus Project as spindle motor
/*! Maxon RE 40 with 150W
 * Voltage: 48V
 */
struct PegasusSpindleMotor : DeviceEPOS2MotorParameters {
	PegasusSpindleMotor(){
         gearratio_motor = 1.0;
         encoder_pulse_number = 1024;
         encoder_qc_number =  4*encoder_pulse_number;
         motor_type  = MOTOR_TYPE_brushed_DC_motor;
         pole_pair_number  = 1;
         thermal_time_constant_winding = 416; //[1/10 s]

         //Default control mode
         actualControlMode = OPERATION_MODE_VELOCITY;

         //Gains:
          velocity_P_Gain = 1241; ///< 0-32767  //default 1000
          velocity_I_Gain = 189;  ///< 0-32767 //default 100
          velocity_VFF_Gain = 0;
          velocity_AFF_Gain = 163;
          current_P_Gain  = 666;    ///< 0-32767 // default 800
          current_I_Gain  = 152;    ///< 0-32767 // default 200


  		positionLimits[0] =  -300.0; //limits in [rad]
  		positionLimits[1] =  300.0; //For the Knee with gear 2:1

         continuous_current_limit = 3.12;        //[A]
         output_current_limit      = 6.24;       //[A]

         max_profile_velocity =  10.0;// 100.0/(gearratio_motor * RAD_S_TO_RPM);// M_PI/2.0;        //[rad/s] of Gear output
         profile_acceleration    = 32*M_PI;            //[rad/s2]
         profile_decceleration    = 32*M_PI;            //[rad/s2]
         max_following_error     = 0.2;                        //[rad]

         TICKS_TO_RAD = (2*M_PI)/(double)encoder_qc_number ;
         RAD_TO_TICKS =1.0/TICKS_TO_RAD;
         rad_s_Gear_to_rpm_Motor =  gearratio_motor * RAD_S_TO_RPM;
     }


};


// RE 25 with 20W
struct Maxon_RE25_20W_Enc_500_Gear_79 : DeviceEPOS2MotorParameters {
     Maxon_RE25_20W_Enc_500_Gear_79(){

         gearratio_motor = 79.0; //Is this correct?
         encoder_pulse_number = 500;
         encoder_qc_number =        4*encoder_pulse_number;
         motor_type  = MOTOR_TYPE_brushed_DC_motor;
         pole_pair_number  = 1;
         thermal_time_constant_winding = 300; //[1/10 s]

         //Default control mode
         actualControlMode = OPERATION_MODE_VELOCITY;
     //    actualControlMode = OPERATION_MODE_CURRENT;

         //Gains:
          velocity_P_Gain = 1000; ///< 0-32767  //default 1000
          velocity_I_Gain = 200;  ///< 0-32767 //default 100
          current_P_Gain  = 800;    ///< 0-32767 // default 800
          current_I_Gain  = 200;    ///< 0-32767 // default 200


  		positionLimits[0] =  -300.0; //limits in [rad]
  		positionLimits[1] =  300.0; //For the Knee with gear 2:1

         continuous_current_limit = 1.0        ;                 //[A]
         output_current_limit      = 1.5;                    //[A]

         max_profile_velocity =  10.0;// 100.0/(gearratio_motor * RAD_S_TO_RPM);// M_PI/2.0;        //[rad/s] of Gear output
         profile_acceleration    = 32*M_PI;            //[rad/s2]
         profile_decceleration    = 32*M_PI;            //[rad/s2]
         max_following_error     = 0.2;                        //[rad]

         TICKS_TO_RAD = (2*M_PI)/(double)encoder_qc_number ;
         RAD_TO_TICKS =1.0/TICKS_TO_RAD;
         rad_s_Gear_to_rpm_Motor =  gearratio_motor * RAD_S_TO_RPM;
     }


};

//! Motor used in project StarlETH
/*! Maxon EC 4pole 200W
 *	Voltage: 48V
 */
struct Maxon_EC_4pole_200_Enc_500 : DeviceEPOS2MotorParameters {
	Maxon_EC_4pole_200_Enc_500(int rxPDOSMId, int txPDOSMId, int inSDOSMId, int outSDOSMId):
		DeviceEPOS2MotorParameters(rxPDOSMId, txPDOSMId, inSDOSMId, outSDOSMId){
		gearratio_motor = 10.0;
		encoder_pulse_number = 500;		// CG ok
		encoder_qc_number =		4.0*encoder_pulse_number;
		encoder_type = 1;
		encoder_polarity = 0;
		hall_polarity = 0;
		motor_type  = MOTOR_TYPE_EC_motor_sinus; // CG ok
		pole_pair_number  = 2;
		thermal_time_constant_winding = 21; //8; //[1/10 s]

		//Default control mode
		actualControlMode = OPERATION_MODE_VELOCITY;

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


		continuous_current_limit = 9.4; //4.7; // 4.7=CG ok				//[A]
		output_current_limit	  = 9.4; //18.8; //9.4;					//[A]

		max_profile_velocity =    M_PI/2.0;		//[rad/s] of Gear output
		profile_acceleration	= 32*M_PI;			//[rad/s2]
		profile_decceleration	= 32*M_PI;			//[rad/s2]
		max_following_error 	= 0.2;						//[rad]

		TICKS_TO_RAD = (2*M_PI)/(double)encoder_qc_number ;
		RAD_TO_TICKS =1.0/TICKS_TO_RAD;
		rad_s_Gear_to_rpm_Motor =  gearratio_motor * RAD_S_TO_RPM;


	}
};



struct MotorHAA : Maxon_EC_4pole_200_Enc_500 {
	MotorHAA(int rxPDOSMId, int txPDOSMId, int inSDOSMId, int outSDOSMId):
		Maxon_EC_4pole_200_Enc_500(rxPDOSMId, txPDOSMId, inSDOSMId, outSDOSMId)
	{
		gearratio_motor = 100.0; //5.0/4.0*80.0;
		encoder_pulse_number = 500;		// CG ok
		encoder_qc_number =		4.0*encoder_pulse_number;
		encoder_polarity = 0;

		//Limits: Dangerous: Assuming that robot home position corresponds to maxon zero position
		positionLimits[0] =  -300.0; //limits in [rad]
		positionLimits[1] =  300.0; //For the Knee with gear 2:1

		//Overwrite generic RE_20 settings for the knee joint:
		max_profile_velocity = 1.0;   //M_PI;		//[rad/s] of Gear output


		TICKS_TO_RAD = (2*M_PI)/(double)encoder_qc_number ;
		RAD_TO_TICKS =1.0/TICKS_TO_RAD;
		rad_s_Gear_to_rpm_Motor =  gearratio_motor * RAD_S_TO_RPM;



	}
};


struct MotorHFE : Maxon_EC_4pole_200_Enc_500 {
	MotorHFE(int rxPDOSMId, int txPDOSMId, int inSDOSMId, int outSDOSMId):
		Maxon_EC_4pole_200_Enc_500(rxPDOSMId, txPDOSMId, inSDOSMId, outSDOSMId)
	{
		gearratio_motor = 100.0; //5.0/4.0*80.0;
		encoder_pulse_number = 500;		// CG ok
		encoder_qc_number =		4.0*encoder_pulse_number;
		encoder_polarity = 0;

		//Limits: Dangerous: Assuming that robot home position corresponds to maxon zero position
		positionLimits[0] =  -300.00; //limits in [rad]
		positionLimits[1] =  300.0; //For the Knee with gear 2:1

		//Overwrite generic RE_20 settings for the knee joint:
		max_profile_velocity =  1.0;//  M_PI;		//[rad/s] of Gear output


		TICKS_TO_RAD = (2*M_PI)/(double)encoder_qc_number ;
		RAD_TO_TICKS =1.0/TICKS_TO_RAD;
		rad_s_Gear_to_rpm_Motor =  gearratio_motor * RAD_S_TO_RPM;


	}
};

struct MotorKFE : Maxon_EC_4pole_200_Enc_500 {
	MotorKFE(int rxPDOSMId, int txPDOSMId, int inSDOSMId, int outSDOSMId):
		Maxon_EC_4pole_200_Enc_500(rxPDOSMId, txPDOSMId, inSDOSMId, outSDOSMId)
	{
		gearratio_motor = 18.0/16.0*100.0; //ok // old=27/16*100
		encoder_pulse_number = 500;		// CG ok
		encoder_qc_number =		4.0*encoder_pulse_number;
		encoder_polarity = 0;

		//Limits: Dangerous: Assuming that robot home position corresponds to maxon zero position
		positionLimits[0] =  -300.00; //limits in [rad]
		positionLimits[1] =  300.0; //For the Knee with gear 2:1

		//Overwrite generic RE_20 settings for the knee joint:
		max_profile_velocity =  1.0; // M_PI;		//[rad/s] of Gear output


		TICKS_TO_RAD = (2*M_PI)/(double)encoder_qc_number ;
		RAD_TO_TICKS =1.0/TICKS_TO_RAD;
		rad_s_Gear_to_rpm_Motor =  gearratio_motor * RAD_S_TO_RPM;


	}
};






#endif
