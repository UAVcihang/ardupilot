/*
 * AC_ESO.h
 *
 *  Created on: 2017-3-20
 *      Author: weihli
 *      Ref: OLD_FC_X att_control
 */

#ifndef AC_ADRC_H_
#define AC_ADRC_H_

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

	//AC_ADRC(float T, float kp, float kd);
	AC_ADRC(float T, float kp, float kd, float r0 = 30000.0f, float b01 = 40.0f, float beta0 = 400.0f, float beta1 = 4000.0f, float beta2 = 8000.0f, int16_t tao_n = 400, int8_t h_n = 10);


	void init(float r0, float b01, float beta0, float beta1, float beta2, int16_t tao_n, int8_t h_n){
		_r0 = r0;
		_b01 = b01;
		_beta0 = beta0;
		_beta1 = beta1;
		_beta2 = beta2;

		if(tao_n <= 0) {
			tao_n = 1;
		}
		_tao = _h0 * tao_n;

		if(h_n <= 0) {
			h_n = 1;
		}
		_h1 = _h0 * h_n;
	}

	float control_adrc(float v,float y,float u,float MAX);

	float get_e1() const{
		return _e1;
	}
	
	float get_e2() const{
		return _e2;
	}
	
	float get_z3() const{
		return _z[2];
	}
	
	float get_z2() const{
		return _z[1];
	}

	float get_z1() const{
		return _z[0];
	}

	float get_x1() const{
		return _v1;
	}

	float get_x2() const{
		return _v2;
	}

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];
protected:
	float fst(float x1,float x2,float w,float h);
	float fal(float e,float alfa,float delta);
	float fst2(float x1,float x2,float w, float h);
	//float sign(float x);
	float eso_3n(float v,float y,float u,float T,float MAX);
	float eso_2n(float v,float y,float u,float T,float MAX);
	float adrc_eso(float v,float y,float u,float T,float MAX);
	void adrc_td(float in);

	int8_t sign_adrc(float input){
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
	// 跟踪器参数
	// 参数
	AP_Float _kp, _kd;
	float _beta0, _beta1, _beta2;
	float _r0; //跟踪速度
	float _b01; //模型增益
	float  _alfa1, _alfa2; // 非线性

	float _tao;
	float _h0; //滤波因子

	//float _r1;
	float _h1;

	float _v1, _v2; // 跟踪微分器状态量及其微分项
	float _z[3]; // 扰动信息

	//float _integer;
	//float _b01;
	float _e1, _e2;
	float _disturb, _disturb_u/*, _disturb_u_reg*/;
	float _u;

	uint8_t  _level;  //系统阶次
	//uint8_t  _n;
	//bool _not_use_px4;
	//bool _use_td;

};

#endif /* AC_ESO_H_ */
