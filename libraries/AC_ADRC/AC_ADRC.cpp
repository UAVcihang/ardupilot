/*
 * AC_ESO.cpp
 *
 *  Created on: 2017-3-20
 *      Author: weihli
 *      Ref: OLD_X_FC att_control
 */
#include <AP_Math/AP_Math.h>
#include "AC_ADRC.h"

const AP_Param::GroupInfo AC_ADRC::var_info[] = {
	    // @Param: P
	    // @DisplayName: PID Proportional Gain
	    // @Description: P Gain which produces an output value that is proportional to the current error value
	    AP_GROUPINFO("P",    0, AC_ADRC, _kp, 0),


	    // @Param: D
	    // @DisplayName: PID Derivative Gain
	    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
	    AP_GROUPINFO("D",    1, AC_ADRC, _kd, 0),

    AP_GROUPEND
};


AC_ADRC::AC_ADRC(float T, float kp, float kd, float r0, float b01, float beta0, float beta1, float beta2, int16_t tao_n, int8_t h_n) :
		_r0(r0),
		_b01(b01),
		_beta0(beta0),
		_beta1(beta1),
		_beta2(beta2)

{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

	_h0 = T;

	_kp = kp * 18.0f; //kp * 10.0f;
	_kd = kd * 0.5f; //kd;

	// _alfa0 = 0.25f;
	_alfa1 = 0.75f;
	_alfa2 = 1.5f;
	//_tao = _h0 * 1000.0f;
	if(tao_n <= 0)
		tao_n = 1;
	_tao  = _h0 * tao_n;//_h0 * 400.0f; // too low will cause oscillation

	//_r1 = 0.5f / (_h0 * _h0);
	//_h1 = _h0 * 10.0f;
	if(h_n <=0)
		h_n = 1;
	_h1 = _h0 * h_n;//_h0 * 10.0f;

	/*_r0 = 10000.0f;
	_beta0 = 400.0f;
	_beta1 = 9000.0f;
	_beta2 = 10000.0f;
	_b01 = 1000.0f;*/
	
	_v1 = _v2 = 0.0f;

	//_integer = 0.0f;
	_e1 = _e2 = 0.0f;

	//_b01 = 40;//200.0f;// 200 40.0f; // if no disturb value to 40

	_disturb_u = _disturb = 0.0f;
	_level = 2;
}


/*AC_ADRC::AC_ADRC(float T, float kp, float kd)

	{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);
		_h0 = T;
		//_r0 = 30000;
		//_beta0 = 400.0f;//1.0f / (_h0 + 0.000001f);
		//_beta1 = 4000.0f;//1.0f / (30.0f * _h0 * _h0 + 0.000001f);
		//_beta2 = 8000.0f;//10000.0f;

		_kp = kp * 10.0f;
		//_ki = 0.0f;
		_kd = kd;

		// _alfa0 = 0.25f;
		//_alfa1 = 0.75f;
		// _alfa2 = 1.5f;
		_tao  = _h0 * _tao_n;//_h0 * 400.0f; // too low will cause oscillation

		//_c = 0.5f;
		//_r1 = 0.5f / (_h0 * _h0);
		_h1 = _h0 * _h1_n;//_h0 * 10.0f;

		_v1 = _v2 = 0.0f;

		//_integer = 0.0f;
		_e1 = _e2 = 0.0f;

		//_b01 = 40;//200.0f;// 200 40.0f; // if no disturb value to 40

		_level = 2;
	}*/

// adrc Î¢·Ö¸ú×ÙÆ÷TD£¬
void AC_ADRC::adrc_td(float in)
{
	float v1_pre = _v1;
	_v1 += _h0 * _v2;                        //td_x1=v1; v1(t+1) = v1(t) + T * v2(t)
	_v2 += _h0 * fst2(v1_pre - in, _v2, _r0, _h1);           //td_x2=v2 v2(t+1) = t2(t) + T fst2(v1(t) - in, v2(t), r, h)
}

// fhanº¯Êý
float AC_ADRC::fst2(float x1, float x2, float w, float h)
{

	float td_y=0;
	float a0=0,a1,a2;
	float a=0;
	float fhan=0;
	float d=0;
	float d0=0;//dead
	float sy,sa;

	d = w * h * h;
	a0 = h * x2;
	td_y = x1 + a0;
	a1 = sqrtf(d*(d + 8.0 * fabs(td_y)));
	a2 = a0 + sign_adrc(td_y) * (a1-d) * 0.5f;
	sy = (sign_adrc(td_y + d) - sign_adrc(td_y - d)) * 0.5f;
	a = (a0 + td_y - a2) * sy + a2;
	sa = (sign_adrc(a + d) - sign_adrc(a - d)) * 0.5f;
	fhan = -w * (a / d - sign_adrc(a)) * sa - w * sign_adrc(a);
	return(fhan);
}

float AC_ADRC::fal(float e,float alfa,float delta)
{
	float y = 0.0f;
	if(fabs(e) > delta){
		y = pow(fabs(e), alfa) * sign_adrc(e);
	}
	else{
		y=e / (pow(delta, 1.0 - alfa) + 0.0000001f);
	}
	return(y);
}


//------------------------------ESO--------------------------------
float AC_ADRC::eso_2n(float v,float y,float u,float T,float MAX)
{
	float e=0,fe,fe1;
	//_h = _h0;
    //********  eso  *************

	/*if(_auto_b0==0)
		_b01=_b0;*/
	e = _z[0] - y;
	fe = fal(e, 0.5, _h0);
	fe = fe * 1.0f;
	fe1 = fal(e, 0.25, _h0);
	fe1 = fe1 * 1.0f;
	_z[0] += _h0 * (_z[1] - _beta0 * e + _b01 *u);
	_z[1] += -_h0 * _beta1 * e;

	return _disturb = constrain_float(_z[1], -MAX, MAX);
}

float AC_ADRC::eso_3n(float v,float y,float u,float T,float MAX)
{
	float e=0,fe,fe1;

//********  eso  *************
	e = _z[0] - y;
	fe = fal(e, 0.5f, _h0);
	fe1=fal(e, 0.25f, _h0);
	_z[0] += _h0 * (_z[1] - _beta0*e);
	_z[1] += _h0 * (_z[2] - _beta1 * fe + _b01 *  u);
	_z[2] += -_h0 * _beta2 * fe1;

	return _disturb=constrain_float(_z[2] , -MAX, MAX);
}

float AC_ADRC::adrc_eso(float v,float y,float u,float T,float MAX)
{

	_e1 = _v1 - _z[0];
	_e2 = _v2 - _z[1];
	//_e0 += _e;//*T;

	_u = _kp * fal(_e1, _alfa1, _tao) + _kd * fal(_e2,_alfa2,_tao);
	if(!is_zero(_b01)){
	switch(_level){
		case 1:
			_disturb_u = _z[1] / _b01;
			break;
		case 2:
			_disturb_u = _z[2] / _b01;
			break;
	}


	_u -= _disturb_u;
	}
	return  _u = constrain_float(_u, -MAX, MAX);
}


float AC_ADRC::control_adrc(float v,float y,float u,float MAX)
{
	// TD ¸ú×ÙÎ¢·ÖÆ÷
	adrc_td(v);

	// ESO À©ÕÅ×´Ì¬¹Û²âÆ÷
	switch(_level){
		case 1:eso_2n(v, y, u, _h0, MAX);break;
		case 2:eso_3n(v, y, u, _h0, MAX);break;
	}

	// NLSEF ·ÇÏßÐÔÎó²î·´À¡
	/*_integer += _ki * ( v - y ) * _h0;
	_integer = constrain_float( _integer, -4500.0f, 4500.0f );*/
	adrc_eso(v, y, u, _h0, MAX);
	return _u;
}


