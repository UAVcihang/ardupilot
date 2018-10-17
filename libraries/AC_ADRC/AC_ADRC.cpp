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
		 AP_GROUPINFO_FLAGS("ENB", 1, AC_ADRC, _enable, 0, AP_PARAM_FLAG_ENABLE),
	    // @Param: P
	    // @DisplayName: PID Proportional Gain
	    // @Description: P Gain which produces an output value that is proportional to the current error value
	    AP_GROUPINFO("P",    2, AC_ADRC, _kp, 0),


	    // @Param: D
	    // @DisplayName: PID Derivative Gain
	    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
	    AP_GROUPINFO("D",    3, AC_ADRC, _kd, 0),

	    AP_GROUPINFO("A1",    4, AC_ADRC, _alfa1, 0),

	    AP_GROUPINFO("A2",    5, AC_ADRC, _alfa2, 0),

	    //AP_GROUPINFO("delta",    4, AC_ADRC, _delta, 0),

	    AP_GROUPINFO("BT0",    6, AC_ADRC, _beta0, 0),
	    AP_GROUPINFO("BT1",    7, AC_ADRC, _beta1, 0),
	    AP_GROUPINFO("BT2",    8, AC_ADRC, _beta2, 0),
	    AP_GROUPINFO("R0",    9, AC_ADRC, _r0, 0),
	    AP_GROUPINFO("B0",    10, AC_ADRC, _b01, 0),
	    AP_GROUPINFO("TN",    11, AC_ADRC, _tao_n, 400),
	    AP_GROUPINFO("HN",    12, AC_ADRC, _h_n, 5),
	    AP_GROUPINFO("DTN",    13, AC_ADRC, _delta_n, 400),

    AP_GROUPEND
};


AC_ADRC::AC_ADRC(float T, float kp, float kd, float a1, float a2, float r0, float b01, float beta0, float beta1, float beta2, int16_t tao_n, int16_t delta_n, int16_t h_n) //:
		//_beta0(beta0),
		//_beta1(beta1),
		//_beta2(beta2),
		//_r0(r0),
		//_b01(b01)
		//_alfa1(a1),
		//_alfa2(a2),
		//_delta(delta)

{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

	_h0 = T;

	_kp = kp ; //kp * 10.0f;
	_kd = kd; //kd;

	_alfa1 = a1;
	_alfa2 = a2;
	//_delta = delta;
	_r0 = r0;
	_b01 = b01;
	_beta0 = beta0;
	_beta1 = beta1;
	_beta2 = beta2;

	_h_n = h_n;
	_tao_n = tao_n;
	_delta_n = delta_n;

	// _alfa0 = 0.25f;
	//_alfa1 = 0.75f;
	//_alfa2 = 1.5f;
	//_tao = _h0 * 1000.0f;
	if(tao_n <= 0)
		tao_n = 1;
	_tao_n = tao_n;
	_tao  = _h0 * tao_n;//_h0 * 400.0f; // too low will cause oscillation


	if(h_n <=0)
		h_n = 1;
	_h_n = h_n;
	_h1 = _h0 * h_n;//_h0 * 10.0f;

	if(delta_n <=0)
		delta_n = 1;
	_delta_n = delta_n;
	_delta = _h0 * delta_n;

	_v1 = _v2 = 0.0f;

	//_integer = 0.0f;
	_e0 = _e1 = _e2 = 0.0f;

	_eso_error = 0.0f;
	//_b01 = 40;//200.0f;// 200 40.0f; // if no disturb value to 40

	_disturb_u = _disturb = 0.0f;
	_u = 0;
	//_level = 2;
}

/************************************************************************************************************************************
*鍑芥暟鍘熷瀷锛歠loat AC_ADRC::adcr_constrain(float val, float min, float max)
*鍑芥暟鍔熻兘锛氳寖鍥撮檺鍒�
*淇敼鏃ユ湡锛�2018-6-25
*澶�   娉細
*************************************************************************************************************************************/
float AC_ADRC::adrc_constrain(float val, float min, float max)
{
	return (val < min) ? min : ((val > max) ? max : val);
}

/************************************************************************************************************************************
*鍑芥暟鍘熷瀷锛歷oid AC_ADRC::adrc_td(float in)
*鍑芥暟鍔熻兘锛氬井鍒嗚窡韪櫒TD鐨勬眰瑙�,杩欎釜绯荤粺绉颁负蹇�熺鏁ｇ郴缁熸淳鐢熺殑鏈�閫熺鏁ｈ窡韪井鍒嗗櫒
*淇敼鏃ユ湡锛�2018-6-25
*澶�   娉細fh=fhan(x1(k)-v(k),x2(k),r,h)
*************************************************************************************************************************************/
void AC_ADRC::adrc_td(float in)
{
	float v1_pre = _v1;
	_v1 += _h0 * _v2;                                        //td_x1=v1; v1(t+1) = v1(t) + T * v2(t)
	_v2 += _h0 * fhan(v1_pre - in, _v2, _r0, _h1);           //td_x2=v2 v2(t+1) = t2(t) + T fst2(v1(t) - in, v2(t), r, h)
}

/************************************************************************************************************************************
*鍑芥暟鍘熷瀷锛歠loat AC_ADRC::fsg(float x,float d)
*鍑芥暟鍔熻兘锛氭眰瑙ｆ渶浼樻帶鍒跺嚱鏁癴han鐨勪腑闂磋繃娓″彉閲�
*淇敼鏃ユ湡锛�2018-6-27
*澶�   娉細
*************************************************************************************************************************************/
float AC_ADRC::fsg(float x,float d)
{
    float value;
    value=(sign_adrc(x+d)-sign_adrc(x-d))*0.5f;
	return(value);
}
/************************************************************************************************************************************
*鍑芥暟鍘熷瀷锛歠loat AC_ADRC::fhan(float x1, float x2, float w, float h)
*鍑芥暟鍔熻兘锛氭眰瑙ｆ渶浼樻帶鍒跺嚱鏁癴han锛屽師鐐归檮杩戞湁杩炵嚎娈电殑杩炵画骞傚嚱鏁�-----u=fhan(x1,x2,r,h)
*淇敼鏃ユ湡锛�2018-6-27
*澶�   娉細r锛氳〃绀烘闀�,鍐冲畾璺熻釜蹇參鐨勫弬鏁帮紝绉颁负閫熷害鍥犲瓙锛�
*        h锛氫负杈撳叆淇″彿琚櫔澹版薄鏌撴椂锛屽喅瀹氭护娉㈡晥鏋滅殑鍙傛暟锛岀О涓烘护娉㈠洜瀛�
*************************************************************************************************************************************/
float AC_ADRC::fhan(float x1, float x2, float r, float h)
{
	float y=0;
	float a0=0,a1,a2;
	float a=0;
	float fh=0;
	float d=0;
	float sa;

	d = r * h * h;                                             //璁＄畻d=r*h,d=h*d
	a0 = h * x2;                                                //a0=h*x2
	y = x1 + a0;                                                //y=x1+a0
	a1 = sqrtf(d*(d + 8.0 * fabsf(y)));                   //a1=sqrtf(d*(d+8*(|y|)))
	a2 = a0 + sign_adrc(y) * (a1-d) * 0.5f;                     //a2=a0+sign(y)*(a1-d)/2

	a = (a0 + y - a2) * fsg(y,d) + a2;                          //a=(a0+y-a2)*fsg(y,d)+a2
	sa = fsg(a,d);                                              //fsg(a,d)=(sign(a+d)-sign(a-d))/2
	fh = -r * (a / d - sign_adrc(a)) * sa - r * sign_adrc(a); //fhan=-r*fsg(a,d)-r*sign(a)(1-fsg(a,d))
	return(fh);
}


/************************************************************************************************************************************
*鍑芥暟鍘熷瀷锛歠loat AC_ADRC::fal(float e,float alfa,float delta)
*鍑芥暟鍔熻兘锛氭寚鏁板嚱鏁癴al(e,0.5,0.01)
*淇敼鏃ユ湡锛�2018-6-25
*澶�   娉細鍘熺偣闄勮繎鐨勫叿鏈夌嚎鎬ф鐨勮繛缁殑骞傛鍑芥暟
*************************************************************************************************************************************/
float AC_ADRC::fal(float e,float alfa,float delta)
{
	float f = 0.0f;
	float s=0;
	s=fsg(e,delta);
	f=e*s/(powf(delta,1-alfa))+powf(fabsf(e),alfa)*sign_adrc(e)*(1-s);
	return(f);
}

/************************************************************************************************************************************
*鍑芥暟鍘熷瀷锛歠loat AC_ADRC::adrc_eso(float v,float y,float u,float T,float MAX)
*鍑芥暟鍔熻兘锛氬柗娲掑嚱鏁�
*淇敼鏃ユ湡锛�2018-6-25
*澶�   娉細three_hz_loop - 3.3hz loop
*************************************************************************************************************************************/
float AC_ADRC::adrc_eso(float v,float y,float u,float T,float MAX)
{
	float fe,fe1;
	_eso_error = _z[0] - y;
	fe = fal(_eso_error, 0.5f, /*_h0*/_delta);
	fe1=fal(_eso_error, 0.25f, /*_h0*/_delta);
	_z[0] += _h0 * (_z[1] - _beta0*_eso_error);
	_z[1] += _h0 * (_z[2] - _beta1 * fe + _b01 *  u);
	_z[2] += -_h0 * _beta2 * fe1;
	return _disturb=adrc_constrain(_z[2] , -MAX, MAX);//constrain_float

}




/************************************************************************************************************************************
*鍑芥暟鍘熷瀷锛歠loat AC_ADRC::adrc_nsl(float v,float y,float u,float T,float MAX)
*鍑芥暟鍔熻兘锛氭墿灞曠姸鎬佽娴嬪櫒
*淇敼鏃ユ湡锛�2018-6-25
*澶�   娉細three_hz_loop - 3.3hz loop
*************************************************************************************************************************************/
float AC_ADRC::adrc_nsl(float v,float y,float u,float T,float MAX)
{
	_e0+=_e1*_h0;      //e0=e1绉垎
	_e1 = _v1 - _z[0];
	_e2 = _v2 - _z[1];
	_u = _kp * fal(_e1, _alfa1, _tao) + _kd * fal(_e2,_alfa2,_tao); //0<_alfa1<1<_alfa2
	_disturb_u = _z[2] / _b01;
	_u-=_disturb_u;
	_u=adrc_constrain(_u, -MAX, MAX);
	return (_u);

}


/************************************************************************************************************************************
*鍑芥暟鍘熷瀷锛歠loat AC_ADRC::control_adrc(float v,float y,float u,float MAX)
*鍑芥暟鍔熻兘锛歛drc鎺у埗鍣�
*淇敼鏃ユ湡锛�2018-6-25
*澶�   娉細three_hz_loop - 3.3hz loop
*************************************************************************************************************************************/
float AC_ADRC::control_adrc(float v,float y,float u,float MAX)
{
	if(_enable == 0)
		return 0;

	adrc_td(v);                    //杈撳叆璺熻釜寰垎鍣═D
	adrc_eso(v, y, u, _h0, MAX);   //鎵╁睍鐘舵�佽娴嬪櫒ESO
	adrc_nsl(v, y, u, _h0, MAX);   //闈炵嚎鎬ц宸弽棣�
	return _u;
}

/*************************************************************************************************************************
*                              file_end
*************************************************************************************************************************/




//
///*
// * AC_ESO.cpp
// *
// *  Created on: 2017-3-20
// *      Author: weihli
// *      Ref: OLD_X_FC att_control
// */
//#include <AP_Math/AP_Math.h>
//#include "AC_ADRC.h"
//
//const AP_Param::GroupInfo AC_ADRC::var_info[] = {
//	    // @Param: P
//	    // @DisplayName: PID Proportional Gain
//	    // @Description: P Gain which produces an output value that is proportional to the current error value
//	    AP_GROUPINFO("P",    0, AC_ADRC, _kp, 0),
//
//
//	    // @Param: D
//	    // @DisplayName: PID Derivative Gain
//	    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
//	    AP_GROUPINFO("D",    1, AC_ADRC, _kd, 0),
//
//    AP_GROUPEND
//};
//
//
//AC_ADRC::AC_ADRC(float T, float kp, float kd, float r0, float b01, float beta0, float beta1, float beta2, int16_t tao_n, int8_t h_n) :
//		_r0(r0),
//		_b01(b01),
//		_beta0(beta0),
//		_beta1(beta1),
//		_beta2(beta2)
//
//{
//    // load parameter values from eeprom
//    AP_Param::setup_object_defaults(this, var_info);
//
//	_h0 = T;
//
//	_kp = kp * 18.0f; //kp * 10.0f;
//	_kd = kd * 0.5f; //kd;
//
//	// _alfa0 = 0.25f;
//	_alfa1 = 0.75f;
//	_alfa2 = 1.5f;
//	//_tao = _h0 * 1000.0f;
//	if(tao_n <= 0)
//		tao_n = 1;
//	_tao  = _h0 * tao_n;//_h0 * 400.0f; // too low will cause oscillation
//
//	//_r1 = 0.5f / (_h0 * _h0);
//	//_h1 = _h0 * 10.0f;
//	if(h_n <=0)
//		h_n = 1;
//	_h1 = _h0 * h_n;//_h0 * 10.0f;
//
//	/*_r0 = 10000.0f;
//	_beta0 = 400.0f;
//	_beta1 = 9000.0f;
//	_beta2 = 10000.0f;
//	_b01 = 1000.0f;*/
//
//	_v1 = _v2 = 0.0f;
//
//	//_integer = 0.0f;
//	_e1 = _e2 = 0.0f;
//
//	//_b01 = 40;//200.0f;// 200 40.0f; // if no disturb value to 40
//
//	_disturb_u = _disturb = 0.0f;
//	_level = 2;
//}
//
///************************************************************************************************************************************
//*鍑芥暟鍘熷瀷锛歠loat AC_ADRC::adcr_constrain(float val, float min, float max)
//*鍑芥暟鍔熻兘锛氳寖鍥撮檺鍒�
//*淇敼鏃ユ湡锛�2018-6-25
//*澶�   娉細
//*************************************************************************************************************************************/
//float AC_ADRC::adcr_constrain(float val, float min, float max)
//{
//	return (val < min) ? min : ((val > max) ? max : val);
//}
//
//
///************************************************************************************************************************************
//*鍑芥暟鍘熷瀷锛歷oid AC_ADRC::adrc_td(float in)
//*鍑芥暟鍔熻兘锛氬井鍒嗚窡韪櫒TD鐨勬眰瑙�,杩欎釜绯荤粺绉颁负蹇�熺鏁ｇ郴缁熸淳鐢熺殑鏈�閫熺鏁ｈ窡韪井鍒嗗櫒
//*淇敼鏃ユ湡锛�2018-6-25
//*澶�   娉細fh=fhan(x1(k)-v(k),x2(k),r,h)
//*************************************************************************************************************************************/
//void AC_ADRC::adrc_td(float in)
//{
//	float v1_pre = _v1;
//	_v1 += _h0 * _v2;                               //td_x1=v1; v1(t+1) = v1(t) + T * v2(t)
//	_v2 += _h0 * fhan(v1_pre - in, _v2, _r0, _h1);
//	//_v2 += _h0 * fst2(v1_pre - in, _v2, _r0, _h1);           //td_x2=v2 v2(t+1) = t2(t) + T fst2(v1(t) - in, v2(t), r, h)
//}
//
///************************************************************************************************************************************
//*鍑芥暟鍘熷瀷锛歠loat AC_ADRC::fsg(float x,float d)
//*鍑芥暟鍔熻兘锛氭眰瑙ｆ渶浼樻帶鍒跺嚱鏁癴han鐨勪腑闂磋繃娓″彉閲�
//*淇敼鏃ユ湡锛�2018-6-27
//*澶�   娉細
//*************************************************************************************************************************************/
//float AC_ADRC::fsg(float x,float d)
//{
//    float value;
//    value=(sign_adrc(x+d)-sign_adrc(x-d))*0.5f;
//	return(value);
//}
///************************************************************************************************************************************
//*鍑芥暟鍘熷瀷锛歠loat AC_ADRC::fhan(float x1, float x2, float w, float h)
//*鍑芥暟鍔熻兘锛氭眰瑙ｆ渶浼樻帶鍒跺嚱鏁癴han锛屽師鐐归檮杩戞湁杩炵嚎娈电殑杩炵画骞傚嚱鏁�-----u=fhan(x1,x2,r,h)
//*淇敼鏃ユ湡锛�2018-6-27
//*澶�   娉細r锛氳〃绀烘闀�,鍐冲畾璺熻釜蹇參鐨勫弬鏁帮紝绉颁负閫熷害鍥犲瓙锛�
//*        h锛氫负杈撳叆淇″彿琚櫔澹版薄鏌撴椂锛屽喅瀹氭护娉㈡晥鏋滅殑鍙傛暟锛岀О涓烘护娉㈠洜瀛�
//*************************************************************************************************************************************/
//float AC_ADRC::fhan(float x1, float x2, float r, float h)
//{
//
//	float y=0;
//	float a0=0,a1,a2;
//	float a=0;
//	float fhan=0;
//	float d=0;
//	float sa;
//
//	d = r * h * h;                                             //璁＄畻d=r*h,d=h*d
//	a0 = h * x2;                                                //a0=h*x2
//	y = x1 + a0;                                                //y=x1+a0
//	a1 = sqrtf(d*(d + 8.0 * (float)fabs(y)));                   //a1=sqrtf(d*(d+8*(|y|)))
//	a2 = a0 + sign_adrc(y) * (a1-d) * 0.5f;                     //a2=a0+sign(y)*(a1-d)/2
//
//	a = (a0 + y - a2) * fsg(y,d) + a2;                          //a=(a0+y-a2)*fsg(y,d)+a2
//	sa = fsg(a,d);                                              //fsg(a,d)=(sign(a+d)-sign(a-d))/2
//	fhan = -r * (a / d - sign_adrc(a)) * sa - r * sign_adrc(a); //fhan=-r*fsg(a,d)-r*sign(a)(1-fsg(a,d))
//	return(fhan);
//}
//
//
//
//float AC_ADRC::fst2(float x1, float x2, float r, float h)
//{
//	float y=0;
//	float a0=0,a1,a2;
//	float a=0;
//	float fhan=0;
//	float d=0;
//	float sa;
//
//	d = r * h * h;                                             //璁＄畻d=r*h,d=h*d
//	a0 = h * x2;                                                //a0=h*x2
//	y = x1 + a0;                                                //y=x1+a0
//	a1 = sqrtf(d*(d + 8.0 * (float)fabs(y)));                   //a1=sqrtf(d*(d+8*(|y|)))
//	a2 = a0 + sign_adrc(y) * (a1-d) * 0.5f;                     //a2=a0+sign(y)*(a1-d)/2
//
//	a = (a0 + y - a2) * fsg(y,d) + a2;                          //a=(a0+y-a2)*fsg(y,d)+a2
//	sa = fsg(a,d);                                              //fsg(a,d)=(sign(a+d)-sign(a-d))/2
//	fhan = -r * (a / d - sign_adrc(a)) * sa - r * sign_adrc(a); //fhan=-r*fsg(a,d)-r*sign(a)(1-fsg(a,d))
//	return(fhan);
////	float td_y=0;
////	float a0=0,a1,a2;
////	float a=0;
////	float fhan=0;
////	float d=0;
////	float d0=0;//dead
////	float sy,sa;
////
////	d = w * h * h;
////	a0 = h * x2;
////	td_y = x1 + a0;
////	a1 = sqrtf(d*(d + 8.0 * fabs(td_y)));
////	a2 = a0 + sign_adrc(td_y) * (a1-d) * 0.5f;
////	sy = (sign_adrc(td_y + d) - sign_adrc(td_y - d)) * 0.5f;
////	a = (a0 + td_y - a2) * sy + a2;
////	sa = (sign_adrc(a + d) - sign_adrc(a - d)) * 0.5f;
////	fhan = -w * (a / d - sign_adrc(a)) * sa - w * sign_adrc(a);
////	return(fhan);
//}
//
//float AC_ADRC::fal(float e,float alfa,float delta)
//{
//	float fal = 0.0f;
//	float s=0;
//	s=fsg(e,delta);
//	fal=e*s/(powf(delta,1-alfa))+powf(fabs(e),alfa)*sign_adrc(e)*(1-s);
//	return(fal);
////	float y = 0.0f;
////	if(fabs(e) > delta){
////		y = pow(fabs(e), alfa) * sign_adrc(e);
////	}
////	else{
////		y=e / (pow(delta, 1.0 - alfa) + 0.0000001f);
////	}
////	return(y);
//}
//
//
////------------------------------ESO--------------------------------
//float AC_ADRC::eso_2n(float v,float y,float u,float T,float MAX)
//{
//	float e=0,fe,fe1;
//	//_h = _h0;
//    //********  eso  *************
//
//	/*if(_auto_b0==0)
//		_b01=_b0;*/
//	e = _z[0] - y;
//	fe = fal(e, 0.5, _h0);
//	fe = fe * 1.0f;
//	fe1 = fal(e, 0.25, _h0);
//	fe1 = fe1 * 1.0f;
//	_z[0] += _h0 * (_z[1] - _beta0 * e + _b01 *u);
//	_z[1] += -_h0 * _beta1 * e;
//
//	return _disturb = adcr_constrain(_z[1], -MAX, MAX);
//}
//
//float AC_ADRC::eso_3n(float v,float y,float u,float T,float MAX)
//{
//	float e=0,fe,fe1;
//
////********  eso  *************
//	e = _z[0] - y;
//	fe = fal(e, 0.5f, _h0);
//	fe1=fal(e, 0.25f, _h0);
//	_z[0] += _h0 * (_z[1] - _beta0*e);
//	_z[1] += _h0 * (_z[2] - _beta1 * fe + _b01 *  u);
//	_z[2] += -_h0 * _beta2 * fe1;
//
//	return _disturb=adcr_constrain(_z[2] , -MAX, MAX);
//}
//
//float AC_ADRC::adrc_eso(float v,float y,float u,float T,float MAX)
//{
//
//	_e1 = _v1 - _z[0];
//	_e2 = _v2 - _z[1];
//	//_e0 += _e;//*T;
//
//	_u = _kp * fal(_e1, _alfa1, _tao) + _kd * fal(_e2,_alfa2,_tao);
//	if(!is_zero(_b01)){
//	switch(_level){
//		case 1:
//			_disturb_u = _z[1] / _b01;
//			break;
//		case 2:
//			_disturb_u = _z[2] / _b01;
//			break;
//	}
//
//
//	_u -= _disturb_u;
//	}
//	return  _u = adcr_constrain(_u, -MAX, MAX);
//}
//
//
//float AC_ADRC::control_adrc(float v,float y,float u,float MAX)
//{
//	// TD 锟斤拷锟斤拷微锟斤拷锟斤拷
//	adrc_td(v);
//
//	// ESO 锟斤拷锟斤拷状态锟桔诧拷锟斤拷
//	switch(_level){
//		case 1:eso_2n(v, y, u, _h0, MAX);break;
//		case 2:eso_3n(v, y, u, _h0, MAX);break;
//	}
//
//	// NLSEF 锟斤拷锟斤拷锟斤拷锟斤拷罘达拷锟�
//	/*_integer += _ki * ( v - y ) * _h0;
//	_integer = constrain_float( _integer, -4500.0f, 4500.0f );*/
//	adrc_eso(v, y, u, _h0, MAX);
//	return _u;
//}
//
//
