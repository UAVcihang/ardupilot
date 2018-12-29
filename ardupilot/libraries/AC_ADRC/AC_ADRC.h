/*
 * AC_ESO.h
 *
 *  Created on: 2017-3-20
 *      Author: weihli
 *      Ref:
 */

#ifndef AC_ADRC_H_
#define AC_ADRC_H_
//#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

//#define ESO_SIGN(x) (((x) > 0)? (1): (-1))
/*typedef struct
{
	float eso_dead;
	float beta0, beta1, beta2, beta3;
	float disturb, disturb_u, disturb_u_reg;
	float alfa1, alfa2, alfa0, tao, KP, KD, KI, e;
	float z[3];
	float h, integer;
	float v1, v2, h0, r0, b0, b01, h1, r1, c, u;

	uint8_t init, level;
	uint8_t n, out_mode, use_td;
	uint8_t not_use_px4;
	uint8_t auto_b0;

}ESO;*/

class AC_ADRC {
public:

	AC_ADRC(float T, float kp, float kd, float a1, float a2, float r0 = 30000.0f, float b01 = 9000.0f, float beta0 = 400.0f, float beta1 = 4000.0f, float beta2 = 8000.0f, int16_t tao_n = 400, int16_t delta_n = 400, int16_t h_n = 5);

	void init()
	{
		/*_r0 = r0;
		_b01 = b01;
		_beta0 = beta0;
		_beta1 = beta1;
		_beta2 = beta2;*/
		if(_enable == 0)
			return;

		if(_tao_n.get() <= 0)
		{
			_tao_n.set_and_save(1);
		}
		_tao = _h0 * _tao_n.get();

		if(_h_n.get() <= 0) {
			_h_n.set_and_save(1);
		}
		_h1 = _h0 * _h_n.get();

		if(_delta_n.get() <= 0) {
			_delta_n.set_and_save(1);
		}
		_delta = _h0 * _delta_n.get();
	}

	void reset(){
		_e0 = _e1 =  _e2 = 0;
		_eso_error = 0.0f;
		_disturb_u = _disturb = 0.0f;
		_u = 0;
	}

	int8_t get_enable(void) {
		return _enable.get();
	}

	float adrc_constrain(float val, float min, float max);
	float control_adrc(float v,float y,float u,float MAX);

	float get_e1() const
	{
		return _e1;
	}

	float get_e2() const
	{
		return _e2;
	}

	float get_z3() const
	{
		return _z[2];
	}

	float get_z2() const
	{
		return _z[1];
	}

	float get_z1() const
	{
		return _z[0];
	}

	float get_x1() const
	{
		return _v1;
	}

	float get_x2() const
	{
		return _v2;
	}

	float get_eso_error() const
	{
		return _eso_error;
	}

	float get_u() const
	{
		return _u;
	}

    static const struct AP_Param::GroupInfo var_info[];
protected:
	float fst(float x1,float x2,float w,float h);
	float fal(float e,float alfa,float delta);
	float fhan(float x1,float x2,float r, float h);
	float fsg(float x,float d);

	float adrc_eso(float v,float y,float u,float T,float MAX);
	float adrc_nsl(float v,float y,float u,float T,float MAX);
	void adrc_td(float in);

	int8_t sign_adrc(float input) //adrc鐨勭鍙峰嚱鏁�
	{
		//int8_t output
		if(input > 1e-6) {
			return 1;
		}
		else if(input < -1e-6) {
			return -1;
		}
		else
			return 0;
	}

	AP_Int8 _enable;
	AP_Float _kp, _kd;
	AP_Float _alfa1, _alfa2;
	//AP_Float _delta;
	AP_Float _beta0, _beta1, _beta2;
	AP_Float _r0; //锟斤拷锟斤拷锟劫讹拷
	AP_Float _b01; //模锟斤拷锟斤拷锟斤拷
	//float  _alfa1, _alfa2; // 锟斤拷锟斤拷锟斤拷

	AP_Int16  _tao_n;
	AP_Int16  _h_n;
	AP_Int16  _delta_n;

	float _tao;
	float _h1;
	float _h0; //锟剿诧拷锟斤拷锟斤拷
	float _delta;
	float _v1, _v2; // 锟斤拷锟斤拷微锟斤拷锟斤拷状态锟斤拷锟斤拷锟斤拷微锟斤拷锟斤拷
	float _z[3]; // 锟脚讹拷锟斤拷息

	float _e0,_e1, _e2;
	float _eso_error;
	float _disturb, _disturb_u/*, _disturb_u_reg*/;
	float _u;
	//uint8_t  _level;
};
#endif





