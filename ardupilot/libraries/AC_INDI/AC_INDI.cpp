/*
 * AC_INDI.cpp
 *
 *  Created on: 2018-8-2
 *      Author: liwh1
 */
#include "AC_INDI.h"
#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL &hal;
// table of user settable parameters
const AP_Param::GroupInfo AC_INDI::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable
    // @Description: Enable notch filter
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AC_INDI, _enable, 0, AP_PARAM_FLAG_ENABLE),

    AP_GROUPINFO("adpt", 2, AC_INDI, _adaptive, 0),
    AP_GROUPINFO("mu", 3, AC_INDI, _mu, 0.0001f),
    // @Param: FREQ
    // @DisplayName: Frequency
    // @Description: Notch center frequency in Hz
    // @Range: 10 200
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("g1_p", 4, AC_INDI, _g1_p, 0.017837/*0.044f*/),

    // @Param: BW
    // @DisplayName: Bandwidth
    // @Description: Notch bandwidth in Hz
    // @Range: 5 50
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("g1_q", 5, AC_INDI, _g1_q, 0.018315f/*0.048f*/),

    // @Param: ATT
    // @DisplayName: Attenuation
    // @Description: Notch attenuation in dB
    // @Range: 5 30
    // @Units: dB
    // @User: Advanced
    AP_GROUPINFO("g1_r", 6, AC_INDI, _g1_r, 0.001049f/*0.016f*/),

    AP_GROUPINFO("g2_r", 7, AC_INDI, _g2_r, 0.076486f/*0.0005f*/),

    AP_GROUPINFO("err_p", 8, AC_INDI, _err_p, 100.0f/*65.0f*/),
    AP_GROUPINFO("err_q", 9, AC_INDI, _err_q, 100.0f/*65.0f*/),
    AP_GROUPINFO("err_r", 10, AC_INDI, _err_r, 100.0f/*65.0f*/),

    AP_GROUPINFO("rate_p", 11, AC_INDI, _rate_p, 14.0f/*10.0f*/),
    AP_GROUPINFO("rate_q", 12, AC_INDI, _rate_q, 14.0f/*10.0f*/),
    AP_GROUPINFO("rate_r", 13, AC_INDI, _rate_r, 14.0f/*10.0f*/),

    AP_GROUPINFO("act_p", 14, AC_INDI, _act_p, 0.04f),
    AP_GROUPINFO("act_q", 15, AC_INDI, _act_q, 0.04f),
    AP_GROUPINFO("act_r", 16, AC_INDI, _act_r, 0.04f),

    AP_GROUPEND
};

///////////////////////////////////////////INDI///////////////////////////////////////////////////////
/*PARAM_DEFINE_INT32(MC_USE_INDI,1);
PARAM_DEFINE_INT32(MC_INDI_ADAPTIVE,0);
PARAM_DEFINE_FLOAT(MC_INDI_MU,0.1f);

PARAM_DEFINE_FLOAT(MC_INDI_G1_P,0.044f);
PARAM_DEFINE_FLOAT(MC_INDI_G1_Q,0.048f);
PARAM_DEFINE_FLOAT(MC_INDI_G1_R,0.016f);
PARAM_DEFINE_FLOAT(MC_INDI_G2_R,0.0005f);

PARAM_DEFINE_FLOAT(MC_INDI_ERROR_P,65.0f);
PARAM_DEFINE_FLOAT(MC_INDI_ERROR_Q,65.0f);
PARAM_DEFINE_FLOAT(MC_INDI_ERROR_R,65.0f);

PARAM_DEFINE_FLOAT(MC_INDI_RATE_P,10.0f);
PARAM_DEFINE_FLOAT(MC_INDI_RATE_Q,10.0f);
PARAM_DEFINE_FLOAT(MC_INDI_RATE_R,10.0f);

PARAM_DEFINE_FLOAT(MC_INDI_ACT_P,0.35f);
PARAM_DEFINE_FLOAT(MC_INDI_ACT_Q,0.35f);
PARAM_DEFINE_FLOAT(MC_INDI_ACT_R,0.35f);*/

AC_INDI::AC_INDI(/*const AP_AHRS_View& ahrs,*/ const AP_Motors& motors, uint16_t sample_freq):
		/*_ahrs(ahrs),*/
		_motors(motors),
		_sample_freq(sample_freq)
{
	AP_Param::setup_object_defaults(this, var_info);
}

void AC_INDI::init()
{
	if(!_enable){
		return;
	}
    //INDI
    indi.max_rate = STABILIZATION_INDI_MAX_RATE;
    indi.attitude_max_yaw_rate = STABILIZATION_INDI_MAX_R;

     indi.g1 = Vector3f(_g1_p, _g1_q, _g1_r);
     indi.g2 = _g2_r;
    /* Estimation parameters for adaptive INDI */
    indi.est.g1 = Vector3f(_g1_p / (float)INDI_EST_SCALE, _g1_q / (float)INDI_EST_SCALE, _g1_r / (float)INDI_EST_SCALE);

    indi.est.g2 = _g2_r / (float)INDI_EST_SCALE;

    //indi.adaptive=(_adaptive.get() == 1);

    indi.reference_acceleration = {
      _err_p,
      _err_q,
      _err_r,
      _rate_p,
      _rate_q,
      _rate_r};

    indi.est.mu = _mu;

    indi.angular_accel_ref.zero();
    indi.u_act_dyn.zero();
    indi.u_in.zero();

    indi_init_filters();
}


/**
 * Function that resets the filters to zeros
 */
void
AC_INDI::indi_init_filters(void) {
    // tau = 1/(2*pi*Fc)
    /*float tau = 1.0/(2.0*M_PI*STABILIZATION_INDI_FILT_CUTOFF);
    float tau_r = 1.0/(2.0*M_PI*STABILIZATION_INDI_FILT_CUTOFF_R);
    float tau_axis[3] = {tau, tau, tau_r};
    float tau_est = 1.0/(2.0*M_PI*STABILIZATION_INDI_ESTIMATION_FILT_CUTOFF);
    float sample_time = 1.0/_sample_freq;*/
    // Filtering of gyroscope and actuators
    /*for(int8_t i=0; i<3; i++) {
      init_butterworth_2_low_pass(&indi.u[i], tau_axis[i], sample_time, 0.0);
      init_butterworth_2_low_pass(&indi.rate[i], tau_axis[i], sample_time, 0.0);
      init_butterworth_2_low_pass(&indi.est.u[i], tau_est, sample_time, 0.0);
      init_butterworth_2_low_pass(&indi.est.rate[i], tau_est, sample_time, 0.0);
    }*/
    indi.u.set_cutoff_frequency(_sample_freq, STABILIZATION_INDI_FILT_CUTOFF);
    indi.rate.set_cutoff_frequency(_sample_freq, STABILIZATION_INDI_FILT_CUTOFF); //gyro data had filtered
    indi.est.u.set_cutoff_frequency(_sample_freq, STABILIZATION_INDI_ESTIMATION_FILT_CUTOFF);
    indi.est.rate.set_cutoff_frequency(_sample_freq, STABILIZATION_INDI_ESTIMATION_FILT_CUTOFF);
  }
/**
 * @brief Update butterworth filter for p, q and r of a FloatRates struct
 *
 * @param filter The filter array to use
 * @param new_values The new values
 */
void
AC_INDI::filter_pqr(LowPassFilter2pVector3f &filter, Vector3f &new_values) {
	filter.apply(new_values);
  /*update_butterworth_2_low_pass(&filter[0], new_values->p);
  update_butterworth_2_low_pass(&filter[1], new_values->q);
  update_butterworth_2_low_pass(&filter[2], new_values->r);*/
}

/**
 * @brief Caclulate finite difference form a filter array
 * The filter already contains the previous values
 *
 * @param output The output array
 * @param filter The filter array input
 */
void
AC_INDI::finite_difference_from_filter(Vector3f &output, LowPassFilter2pVector3f filter) {
  /*for(int8_t i=0; i<3; i++) {
    output[i] = (filter[i].o[0] - filter[i].o[1])*(float)PERIODIC_FREQUENCY;
 //     output[i] = (filter[i].o[0] - filter[i].o[1]) / dt;

  }*/

	output = (filter.get_filter_out_1() - filter.get_filter_out_2()) * (float)_sample_freq;
}

/**
 * @brief Calculate derivative of an array via finite difference
 *
 * @param output[3] The output array
 * @param new[3] The newest values
 * @param old[3] The values of the previous timestep
 */
void
AC_INDI::finite_difference(Vector3f &output, const Vector3f new1, const Vector3f old) {
  /*for(int8_t i=0; i<3; i++) {
    output[i] = (new1[i] - old[i])*(float)PERIODIC_FREQUENCY;
    //  output[i] = (new1[i] - old[i]) / dt;

  }*/
	output = (new1 - old) * (float)_sample_freq;
}

/**
 * This is a Least Mean Squares adaptive filter
 * It estimates the actuator effectiveness online, by comparing the expected
 * angular acceleration based on the inputs with the measured angular
 * acceleration
 */
void
AC_INDI::lms_estimation(const Vector3f &gyro)
{
	//static uint16_t count = 0;
  static struct IndiEstimation *est = &indi.est;
  // Only pass really low frequencies so you don't adapt to noise
  //===========================================
  /*Vector3f rates;
  get_body_rates(&rates);
  struct FloatRates body_rates;
  body_rates.p = rates(0);
  body_rates.q = rates(1);
  body_rates.r = rates(2);*/
  Vector3f rates(gyro.x, gyro.y, gyro.z);

  filter_pqr(est->u, indi.u_act_dyn);
  filter_pqr(est->rate, rates);

  // Calculate the first and second derivatives of the rates and actuators
  //float rate_d_prev[3];
  //float u_d_prev[3];
  Vector3f rate_d_prev(est->rate_d.x, est->rate_d.y, est->rate_d.z);
  Vector3f u_d_prev(est->u_d.x, est->u_d.y, est->u_d.z);
  //float_vect_copy(rate_d_prev, est->rate_d, 3);
  //float_vect_copy(u_d_prev, est->u_d, 3);
  finite_difference_from_filter(est->rate_d, est->rate);
  finite_difference_from_filter(est->u_d, est->u);
  finite_difference(est->rate_dd, est->rate_d, rate_d_prev);
  finite_difference(est->u_dd, est->u_d, u_d_prev);

  // The inputs are scaled in order to avoid overflows
  //float du[3];
  //float_vect_copy(du, est->u_d, 3);
  //float_vect_scale(du, INDI_EST_SCALE, 3);
  Vector3f du(est->u_d.x, est->u_d.y, est->u_d.z);
	du = du * INDI_EST_SCALE;
  est->g1.x = est->g1.x - (est->g1.x * du.x - est->rate_dd.x) * du.x * est->mu;
  est->g1.y = est->g1.y - (est->g1.y * du.y - est->rate_dd.y) * du.y * est->mu;
  float ddu = est->u_dd.z * (float)INDI_EST_SCALE / (float)_sample_freq;
  //float ddu = est->u_dd[2] * (float)INDI_EST_SCALE *dt;
  float error = (est->g1.z * du.z + est->g2 * ddu - est->rate_dd.z);
  est->g1.z = est->g1.z - error * du.z * est->mu / 3.0f;
  est->g2 = est->g2 - error * 1000.0f * ddu * est->mu / 3.0f;

  //the g values should be larger than zero, otherwise there is positive feedback, the command will go to max and there is nothing to learn anymore...
  if (est->g1.x < 0.01f) { est->g1.x = 0.01f; }
  if (est->g1.y < 0.01f) { est->g1.y = 0.01f; }
  if (est->g1.z < 0.01f) { est->g1.z = 0.01f; }
  if (est->g2   < 0.01f) { est->g2 = 0.01f; }

	//count++;

  if (_adaptive >0) {
    //Commit the estimated G values and apply the scaling
    indi.g1.x = est->g1.x * (float)INDI_EST_SCALE;
    indi.g1.y = est->g1.y * (float)INDI_EST_SCALE;
    indi.g1.z = est->g1.z * (float)INDI_EST_SCALE;
    indi.g2   = est->g2 * (float)INDI_EST_SCALE;


	  /*if(count > 400){
		  count = 0;
		  hal.console->printf("g1 p:%.6f, q:%.6f, r:%.6f, g2:%.6f\n", indi.g1.x, indi.g1.y, indi.g1.z, indi.g2);
	  }*/
//    warnx("G1:%f#%f#%f",(double)indi.g1.p,(double)indi.g1.q,(double)indi.g1.r);
//    warnx("G2:%f",(double)indi.g2);
    /*_indi_status.g1_p = indi.g1.p;
    _indi_status.g1_q = indi.g1.q;
    _indi_status.g1_r = indi.g1.r;
    _indi_status.g2_r = indi.g2;
    _indi_status.timestamp = hrt_absolute_time();*/
    /* publish indi status */
    /*if (_indi_adaptive_pub != nullptr) {
        orb_publish(ORB_ID(indi), _indi_adaptive_pub, &_indi_status);

    } else {
        _indi_adaptive_pub = orb_advertise(ORB_ID(indi), &_indi_status);
    }*/
	  
  }
}

void
AC_INDI::stabilization_indi_calc_cmd(const Vector3f &gyro, const Vector3f &att_err, Vector3f &indi_cmd/*, bool rate_control,float dt*/)
{
  // Propagate the filter on the gyroscopes and actuators

    Vector3f rates(gyro.x, gyro.y, gyro.z);
    /*get_body_rates(&rates);
    struct FloatRates body_rates;
    body_rates.p = rates(0);
    body_rates.q = rates(1);
    body_rates.r = rates(2);*/
  filter_pqr(indi.u, indi.u_act_dyn);
  filter_pqr(indi.rate, rates);


		
  // Calculate the derivative of the rates
  finite_difference_from_filter(indi.rate_d, indi.rate);

  //The rates used for feedback are by default the measured rates. If needed they can be filtered (see below)
  //struct FloatRates rates_for_feedback;
  //RATES_COPY(rates_for_feedback, body_rates);
  Vector3f rates_for_feedback(rates.x, rates.y, rates.z);

  //If there is a lot of noise on the gyroscope, it might be good to use the filtered value for feedback.
  //Note that due to the delay, the PD controller can not be as aggressive.
#if STABILIZATION_INDI_FILTER_ROLL_RATE
  rates_for_feedback.p = indi.rate[0].o[0];
#endif
#if STABILIZATION_INDI_FILTER_PITCH_RATE
  rates_for_feedback.q = indi.rate[1].o[0];
#endif
#if STABILIZATION_INDI_FILTER_YAW_RATE
  rates_for_feedback.r = indi.rate[2].o[0];
#endif

  indi.angular_accel_ref.x = indi.reference_acceleration.err_p * att_err.x
                             - indi.reference_acceleration.rate_p * rates_for_feedback.x;

  indi.angular_accel_ref.y = indi.reference_acceleration.err_q * att_err.y
                             - indi.reference_acceleration.rate_q * rates_for_feedback.y;

  //This separates the P and D controller and lets you impose a maximum yaw rate.
  float rate_ref_r = indi.reference_acceleration.err_r * att_err.z/indi.reference_acceleration.rate_r;
  rate_ref_r = constrain_float(rate_ref_r, -indi.attitude_max_yaw_rate, indi.attitude_max_yaw_rate);
  //BoundAbs(rate_ref_r, indi.attitude_max_yaw_rate);
  indi.angular_accel_ref.z = indi.reference_acceleration.rate_r * (rate_ref_r - rates_for_feedback.z);

  /* Check if we are running the rate controller and overwrite */
  /*if(rate_control) {
      //=============================é¥æŽ§å™¨ç›´æŽ¥æŽ§åˆ¶è§’é€Ÿåº¦===========================
    indi.angular_accel_ref.x =  indi.reference_acceleration.rate_p * (_rates_sp(0) - body_rates.p);
    indi.angular_accel_ref.y =  indi.reference_acceleration.rate_q * (_rates_sp(1) - body_rates.q);
    indi.angular_accel_ref.z =  indi.reference_acceleration.rate_r * (_rates_sp(2) - body_rates.r);
  }*/
    //_rates_sp(0) = indi.angular_accel_ref.p;
    //_rates_sp(1) = indi.angular_accel_ref.q;
    //_rates_sp(2) = indi.angular_accel_ref.r;
  //Increment in angular acceleration requires increment in control input
  //G1 is the control effectiveness. In the yaw axis, we need something additional: G2.
  //It takes care of the angular acceleration caused by the change in rotation rate of the propellers
  //(they have significant inertia, see the paper mentioned in the header for more explanation)
  indi.du.x = 1.0f / indi.g1.x * (indi.angular_accel_ref.x - indi.rate_d.x);
  indi.du.y = 1.0f / indi.g1.y * (indi.angular_accel_ref.y - indi.rate_d.y);
  indi.du.z = 1.0f / (indi.g1.z + indi.g2) * (indi.angular_accel_ref.z - indi.rate_d.z + indi.g2 * indi.du.z);

  /* update increment only if motors are providing enough thrust to be effective */
  /*if (_thrust_sp > MIN_TAKEOFF_THRUST) {
      for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
          //Check for
          //roll saturation
          if(i == AXIS_INDEX_ROLL){
              if(_saturation_status.flags.roll_pos){
                indi.du.p = math::min(indi.du.p, 0.0f);
                //warnx("r+");
              }
              if(_saturation_status.flags.roll_neg){
                indi.du.p = math::max(indi.du.p, 0.0f);
                //warnx("r-");
              }
          }else
          //pitch saturation
          if(i == AXIS_INDEX_PITCH){
              if(_saturation_status.flags.pitch_pos){
                indi.du.q = math::min(indi.du.q, 0.0f);
                //warnx("p+");
              }
              if(_saturation_status.flags.pitch_neg){
                indi.du.q = math::max(indi.du.q, 0.0f);
                //warnx("p-");
              }
          }else
           //yaw saturation
          if(i == AXIS_INDEX_YAW){
              if(_saturation_status.flags.yaw_pos){
                indi.du.r = math::min(indi.du.r, 0.0f);
                //warnx("y+");
              }
              if(_saturation_status.flags.yaw_neg){
                indi.du.r = math::max(indi.du.r, 0.0f);
                //warnx("y-");
              }
          }

        }
  }*/
  if(_motors.get_throttle() > 0.2f) {
	  if(_motors.limit.roll_pitch){
		  indi.du.x = MIN(indi.du.x, 0.0f);
		  indi.du.y = MIN(indi.du.y, 0.0f);
	  }
	  if(_motors.limit.yaw) {
		  indi.du.z = MIN(indi.du.z, 0.0f);
	  }
  }
 // ÓÐ´ý´¦Àí
  //add the increment to the total control input
  /*indi.u_in.x = indi.u[0].o[0] + indi.du.x;
  indi.u_in.y = indi.u[1].o[0] + indi.du.y;
  indi.u_in.z = indi.u[2].o[0] + indi.du.z;*/
  indi.u_in = indi.u.get_filter_out_1() + indi.du;

  //bound the total control input
  /*Bound(indi.u_in.p, -4500, 4500);
  Bound(indi.u_in.q, -4500, 4500);
  Bound(indi.u_in.r, -4500, 4500);*/
  indi.u_in.x = constrain_float(indi.u_in.x, -4500, 4500);
  indi.u_in.y = constrain_float(indi.u_in.y, -4500, 4500);
  indi.u_in.z = constrain_float(indi.u_in.z, -4500, 4500);

  //Propagate input filters
  //first order actuator dynamics
  indi.u_act_dyn.x = indi.u_act_dyn.x + _act_p * (indi.u_in.x - indi.u_act_dyn.x);
  indi.u_act_dyn.y = indi.u_act_dyn.y + _act_p * (indi.u_in.y - indi.u_act_dyn.y);
  indi.u_act_dyn.z = indi.u_act_dyn.z + _act_p * (indi.u_in.z - indi.u_act_dyn.z);

  //Don't increment if thrust is off
  //TODO: this should be something more elegant, but without this the inputs
  //will increment to the maximum before even getting in the air.
  /*  if (_thrust_sp < MIN_TAKEOFF_THRUST) {
    FLOAT_RATES_ZERO(indi.du);
    FLOAT_RATES_ZERO(indi.u_act_dyn);
    FLOAT_RATES_ZERO(indi.u_in);
  } else if(_armed.armed){
    // only run the estimation if the commands are not zero.
    lms_estimation(dt);
  }*/
  if(_motors.get_throttle() < 0.2f){
	  indi.du.zero();
	  indi.u_act_dyn.zero();
	  indi.u_in.zero();
  }else if(_motors.armed()) {
	  lms_estimation(gyro);
  }
  /*  INDI feedback */
  /*indi_commands[COMMAND_ROLL] = indi.u_in.x;
  indi_commands[COMMAND_PITCH] = indi.u_in.y;
  indi_commands[COMMAND_YAW] = indi.u_in.z;*/
  indi_cmd =  Vector3f(indi.u_in.x / 4500.0f, indi.u_in.y / 4500.0f, indi.u_in.z / 4500.0f);
}

/**
 * @brief runs stabilization indi
 *
 * @param in_flight not used
 * @param rate_control rate control enabled, otherwise attitude control
 */
void
AC_INDI::stabilization_indi_run(const Vector3f &gyro, const Vector3f &att_err, Vector3f& indi_cmd)
{

	if(!_enable){
		return;
	}
  /*Vector3f att_err;
  get_attitude_errors(&att_err);*/

  /* compute the INDI command */
  stabilization_indi_calc_cmd(gyro, att_err, indi_cmd);


  /* copy the INDI command */
 /* _att_control(AXIS_INDEX_ROLL) = stabilization_att_indi_cmd[COMMAND_ROLL] / 4500.0f;
  _att_control(AXIS_INDEX_PITCH) = stabilization_att_indi_cmd[COMMAND_PITCH] / 4500.0f;
  _att_control(AXIS_INDEX_YAW) = stabilization_att_indi_cmd[COMMAND_YAW] / 4500.0f;*/

}


