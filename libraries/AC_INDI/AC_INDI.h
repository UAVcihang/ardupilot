/*
 * AC_INDI.h
 *
 *  Created on: 2018-8-2
 *      Author: liwh1
 */

#ifndef AC_INDI_H_
#define AC_INDI_H_

#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <Filter/LowPassFilter2p.h>
#include <AP_Motors/AP_Motors.h>          // motors library
//=============================INDI==========================

#define STABILIZATION_INDI_FILT_CUTOFF 3.2f
#define STABILIZATION_INDI_FILT_CUTOFF_R 3.2f
#define STABILIZATION_INDI_ESTIMATION_FILT_CUTOFF 4.0
#define STABILIZATION_INDI_MAX_RATE 6.0f
#define STABILIZATION_INDI_MAX_R 5.23598775f
#define STABILIZATION_INDI_FILTER_ROLL_RATE 0
#define STABILIZATION_INDI_FILTER_PITCH_RATE 0
#define STABILIZATION_INDI_FILTER_YAW_RATE 0
// these parameters are used in the filtering of the angular acceleration
// define them in the airframe file if different values are required
/*#ifndef STABILIZATION_INDI_FILT_CUTOFF
#define STABILIZATION_INDI_FILT_CUTOFF 8.0
#endif

// the yaw sometimes requires more filtering
#ifndef STABILIZATION_INDI_FILT_CUTOFF_R
#define STABILIZATION_INDI_FILT_CUTOFF_R STABILIZATION_INDI_FILT_CUTOFF
#endif*/

/*#ifndef STABILIZATION_INDI_MAX_RATE
#define STABILIZATION_INDI_MAX_RATE 6.0
#endif

#ifndef STABILIZATION_INDI_MAX_R
#define STABILIZATION_INDI_MAX_R STABILIZATION_ATTITUDE_SP_MAX_R
#endif

#ifndef STABILIZATION_INDI_ESTIMATION_FILT_CUTOFF
#define STABILIZATION_INDI_ESTIMATION_FILT_CUTOFF 4.0
#endif*/
//The G values are scaled to avoid numerical problems during the estimation
#define INDI_EST_SCALE 0.001f
#define PERIODIC_FREQUENCY 400

//=============================INDI==========================


class AC_INDI {
public:
	AC_INDI(/*const AP_AHRS_View& ahrs, */const AP_Motors& motors, uint16_t sample_freq);
	void init();

	void stabilization_indi_run(const Vector3f &gyro, const Vector3f &att_err, Vector3f& indi_cmd);
	int8_t get_enable(void) {
		return _enable.get();
	}
	static const struct AP_Param::GroupInfo var_info[];

protected:
	//const AP_AHRS_View &        _ahrs;
	const AP_Motors&            _motors;

	AP_Int8 _enable;
	AP_Int8 _adaptive;

	AP_Float _mu;

	AP_Float _g1_p;
	AP_Float _g1_q;
	AP_Float _g1_r;
	AP_Float _g2_r;

	AP_Float _err_p;
	AP_Float _err_q;
	AP_Float _err_r;

	AP_Float _rate_p;
	AP_Float _rate_q;
	AP_Float _rate_r;

	AP_Float _act_p;
	AP_Float _act_q;
	AP_Float _act_r;


	   //=============================INDI==========================

		/**
		 * @brief angular rates
		 * @details Units: rad/s */
	/*	struct FloatRates {
			float p;
			float q;
			float r;
		};*/

	    struct ReferenceSystem {
	      float err_p;
	      float err_q;
	      float err_r;
	      float rate_p;
	      float rate_q;
	      float rate_r;
	    };

	    struct IndiEstimation {
	    	LowPassFilter2pVector3f u;
	    	LowPassFilter2pVector3f rate;
	      //Butterworth2LowPass u[3];
	      //Butterworth2LowPass rate[3];

	    	Vector3f rate_d;
	    	Vector3f rate_dd;
	    	Vector3f u_d;
	    	Vector3f u_dd;
	    	Vector3f g1;
	      /*float rate_d[3];
	      float rate_dd[3];
	      float u_d[3];
	      float u_dd[3];
	      struct FloatRates g1;*/
	      float g2;
	      float mu;
	    };

	    struct IndiVariables {
	    	Vector3f angular_accel_ref;
	    	Vector3f du;
	    	Vector3f u_in;
	    	Vector3f u_act_dyn;
	    	Vector3f rate_d;
	      /*struct FloatRates angular_accel_ref;
	      struct FloatRates du;
	      struct FloatRates u_in;
	      struct FloatRates u_act_dyn;
	      float rate_d[3];*/



	      LowPassFilter2pVector3f u;
	      LowPassFilter2pVector3f rate;
	      //Butterworth2LowPass u[3];
	      //Butterworth2LowPass rate[3];

	      //struct FloatRates g1;
	      Vector3f g1;
	      float g2;

	      struct ReferenceSystem reference_acceleration;

	      //bool adaptive;             ///< Enable adataptive estimation
	      float max_rate;            ///< Maximum rate in rate control in rad/s
	      float attitude_max_yaw_rate; ///< Maximum yaw rate in atttiude control in rad/s
	      struct IndiEstimation est; ///< Estimation parameters for adaptive INDI
	    };
	    struct IndiVariables indi;

	    uint16_t _sample_freq;
	    //int32_t stabilization_att_indi_cmd[COMMANDS_NB];
	//=============================INDI==========================

	    void indi_init_filters(void);
	    void filter_pqr(LowPassFilter2pVector3f &filter, Vector3f &new_values);
	    void finite_difference_from_filter(Vector3f &output, LowPassFilter2pVector3f filter);
	    void finite_difference(Vector3f &output, const Vector3f new1, const Vector3f old);
	    void lms_estimation(const Vector3f &gyro);
	    void get_body_rate();
	    void stabilization_indi_calc_cmd(const Vector3f &gyro, const Vector3f &att_err, Vector3f& indi_cmd/*, bool rate_control, float dt*/);
};

#endif /* AC_INDI_H_ */
