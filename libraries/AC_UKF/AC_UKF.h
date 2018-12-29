/*
 * AC_UKF.h
 *
 *  Created on: 2018-8-10
 *      Author: liwh1
 */

#ifndef AC_UKF_H_
#define AC_UKF_H_

#include "srcdkf.h"
#include <AP_AHRS/AP_AHRS.h>
//#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>


#define UKF_LOG_SIZE		(17*sizeof(float))
#define UKF_LOG_BUF_SIZE	(UKF_LOG_SIZE*40)
//#define UKF_LOG_FNAME		"UKF"		// comment out to disable logging

#define SIM_S                   10//17		// states
#define SIM_M                   3		// max measurements
#define SIM_V                   9//12//16		// process noise
#define SIM_N                   3		// max observation noise

#define UKF_GYO_AVG_NUM		40

/*#define UKF_STATE_VELN		0
#define UKF_STATE_VELE		1
#define UKF_STATE_VELD		2
#define UKF_STATE_POSN		3
#define UKF_STATE_POSE		4
#define UKF_STATE_POSD		5*/
#define UKF_STATE_ACC_BIAS_X	0//6
#define UKF_STATE_ACC_BIAS_Y	1//7
#define UKF_STATE_ACC_BIAS_Z	2//8
#define UKF_STATE_GYO_BIAS_X	3//9
#define UKF_STATE_GYO_BIAS_Y	4//10
#define UKF_STATE_GYO_BIAS_Z	5//11
#define UKF_STATE_Q1		6//12
#define UKF_STATE_Q2		7//13
#define UKF_STATE_Q3		8//14
#define UKF_STATE_Q4		9//15
//#define UKF_STATE_PRES_ALT	16

#define UKF_V_NOISE_ACC_BIAS_X	0
#define UKF_V_NOISE_ACC_BIAS_Y	1
#define UKF_V_NOISE_ACC_BIAS_Z	2
#define UKF_V_NOISE_GYO_BIAS_X	3
#define UKF_V_NOISE_GYO_BIAS_Y	4
#define UKF_V_NOISE_GYO_BIAS_Z	5
#define UKF_V_NOISE_RATE_X	6
#define UKF_V_NOISE_RATE_Y	7
#define UKF_V_NOISE_RATE_Z	8
//#define UKF_V_NOISE_VELN	9
//#define UKF_V_NOISE_VELE	10
//#define UKF_V_NOISE_VELD	11

/*#define UKF_VELN		navUkfData.x[UKF_STATE_VELN]
#define UKF_VELE		navUkfData.x[UKF_STATE_VELE]
#define UKF_VELD		navUkfData.x[UKF_STATE_VELD]
#define UKF_POSN		navUkfData.x[UKF_STATE_POSN]
#define UKF_POSE		navUkfData.x[UKF_STATE_POSE]
#define UKF_POSD		navUkfData.x[UKF_STATE_POSD]*/
#define UKF_ACC_BIAS_X		_x[UKF_STATE_ACC_BIAS_X]
#define UKF_ACC_BIAS_Y		_x[UKF_STATE_ACC_BIAS_Y]
#define UKF_ACC_BIAS_Z		_x[UKF_STATE_ACC_BIAS_Z]
#define UKF_GYO_BIAS_X		_x[UKF_STATE_GYO_BIAS_X]
#define UKF_GYO_BIAS_Y		_x[UKF_STATE_GYO_BIAS_Y]
#define UKF_GYO_BIAS_Z		_x[UKF_STATE_GYO_BIAS_Z]
#define UKF_Q1			_x[UKF_STATE_Q1]
#define UKF_Q2			_x[UKF_STATE_Q2]
#define UKF_Q3			_x[UKF_STATE_Q3]
#define UKF_Q4			_x[UKF_STATE_Q4]
//#define UKF_PRES_ALT		navUkfData.x[UKF_STATE_PRES_ALT]
/*
#ifdef USE_PRES_ALT
#define UKF_ALTITUDE	UKF_PRES_ALT
#else
#define UKF_ALTITUDE	UKF_POSD
#endif
*/
#define UKF_HIST		40
//#define UKF_P0			101325.0f			    // standard static pressure at sea level

/*#define UKF_FLOW_ROT		-90.0f				    // optical flow mounting rotation in degrees
#define UKF_FOCAL_LENGTH	16.0f				    // 16mm
#define UKF_FOCAL_PX		(UKF_FOCAL_LENGTH / (4.0f * 6.0f) * 1000.0f)   // pixel size: 6um, binning 4 enabled
*/
/*#define UKF_VEL_Q               +3.2545e-02     // +0.032544903471       0.000000350530 +0.000037342305
#define UKF_VEL_ALT_Q           +1.4483e-01     // +0.144827254833       0.000000347510 -0.000055111229
#define UKF_POS_Q               +7.1562e+03     // +7156.240473309331    0.000000352142 +2.727925965284749
#define UKF_POS_ALT_Q           +5.3884e+03     // +5388.369673129109    0.000000351319 -6.187843541372100*/
#define UKF_ACC_BIAS_Q          +1.3317e-03     // +0.001331748045       0.000000359470 +0.000000039113
#define UKF_GYO_BIAS_Q          +4.5256e-02     // +0.045255679186       0.000000349060 +0.000045999290
#define UKF_QUAT_Q              +5.4005e-04     // +0.000540045060       0.000000353882 +0.000000029711
//#define UKF_PRES_ALT_Q          +6.3105e+01     // +63.104671424320      0.000000353790 +0.0166164673283
#define UKF_ACC_BIAS_V          +7.8673e-07     // +0.000000786725       0.000000345847 -0.000000000977
#define UKF_GYO_BIAS_V          +4.0297e-09     // +0.000000004030       0.000000359017 +0.000000000000
#define UKF_RATE_V              +1.7538e-05     // +0.000017538388       0.000000358096 +0.000000000397
//#define UKF_VEL_V               +2.8605e-07     // +0.000000286054       0.000000351709 +0.000000000183
//#define UKF_ALT_VEL_V           +6.8304e-08     // +0.000000068304       0.000000362348 -0.000000000050
//#define UKF_GPS_POS_N           +8.0703e-06     // +0.000008070349       0.000000353490 +0.000000005602
//#define UKF_GPS_POS_M_N         +3.0245e-05     // +0.000030245341       0.000000345021 -0.000000008396
//#define UKF_GPS_ALT_N           +1.1796e-05     // +0.000011795879       0.000000356036 -0.000000010027
//#define UKF_GPS_ALT_M_N         +3.8329e-05     // +0.000038328879       0.000000346581 +0.000000027268
//#define UKF_GPS_VEL_N           +1.7640e-01     // +0.176404763511       0.000000355574 -0.000094105688
//#define UKF_GPS_VEL_M_N         +3.0138e-02     // +0.030138272888       0.000000343584 -0.000002668997
//#define UKF_GPS_VD_N            +4.6379e+00     // +4.637855992835       0.000000358079 +0.000310962082
//#define UKF_GPS_VD_M_N          +1.3127e-02     // +0.013127146795       0.000000347978 -0.000001550944
//#define UKF_ALT_N               +9.5913e-02     // +0.095913477777       0.000000356359 -0.000049781087
#define UKF_ACC_N               +6.3287e-05     // +0.000063286884       0.000000342761 -0.000000022717
#define UKF_DIST_N              +9.7373e-03     // +0.009737270392       0.000000356147 +0.000009059372
#define UKF_MAG_N               +5.2355e-01     // +0.523549973965       0.000000500000 +0.000000000000

//#define NAV_EQUATORIAL_RADIUS	(6378.137 * 1000.0)			    // meters
//#define NAV_FLATTENING		(1.0 / 298.257223563)			    // WGS-84
//#define NAV_E_2			(NAV_FLATTENING * (2.0 - NAV_FLATTENING))

#define IMU_MAG_DECL	(0.0f)
#define IMU_MAG_INCL	(-65.0f)//65

#define RUN_SENSOR_HIST		10
#define IMU_STATIC_STD		0.05f

/*typedef struct {
    srcdkf_t *kf;
    float v0a[3];
    float v0m[3];
    double holdLat, holdLon;
    //double r1, r2;
    //float posN[UKF_HIST];
    //float posE[UKF_HIST];
    //float posD[UKF_HIST];
    //float velN[UKF_HIST];
    //float velE[UKF_HIST];
    //float velD[UKF_HIST];
    int navHistIndex;
    float yaw, pitch, roll;
    float yawCos, yawSin;
    float *x;			// states
    //float flowSumX, flowSumY;
    //int32_t flowSumQuality;
    //float flowSumAlt;
    //float flowVelX, flowVelY;
    //float flowPosN, flowPosE;
    //float flowQuality;
    //float flowAlt;
    //float flowRotCos, flowRotSin;
    //uint32_t flowCount, flowAltCount;
    //int logPointer;
    //volatile uint8_t flowLock;
    //uint8_t flowInit;
    uint8_t logHandle;
} navUkfStruct_t;*/

typedef struct {
    //float bestHacc;
    //float accMask;
    float accHist[3][RUN_SENSOR_HIST];
    float magHist[3][RUN_SENSOR_HIST];
    //float presHist[RUN_SENSOR_HIST];
    float sumAcc[3];
    float sumMag[3];
    //float sumPres;
    int sensorHistIndex;
    //float *altPos;
    //float *altVel;
} runStruct_t;

class AC_UKF{
public:

	//AC_UKF(int8_t s, int8_t m, int8_t v, int8_t n);
	//~AC_UKF();
	AC_UKF(const AP_AHRS* ahrs, AP_Baro &baro, float dt):
	_ahrs(ahrs),
	_dt(dt),
	_loops(0),
	_axis(0)
	//_baro_alt(0.0f),
	//_baro_last_update(0)
	{
		kf = new SRCDKF<AC_UKF>(this);
		kf->setTimeUpdate(&AC_UKF::navUkfTimeUpdate);
	}

	//friend srcdkf_t *srcdkfInit(int s, int m, int v, int n, SRCDKFTimeUpdate_t *timeUpdate);
	// void altUkfInit(void);
	void update(void);
	
	/*float get_height_ukf(){
		return -*_altUkfData.x;
	}*/
	void init(void);
	void navUkfInit(void);
	//friend navUkfStruct_t _navUkfData;

	Vector3f get_attidute(void){
		/*float roll = _navUkfData.roll;
		float pitch = _navUkfData.pitch;
		float yaw = _navUkfData.yaw;*/
		return Vector3f(_roll, _pitch, _yaw);
	}
private:
    const AP_AHRS *_ahrs;
    //AP_Baro &_baro;
    float _dt;
    //float _baro_alt;
    //uint32_t _baro_last_update;
    uint32_t _loops;
    uint32_t _axis;

     float _v0a[3];
     float _v0m[3];

    int _navHistIndex;
    float _yaw, _pitch, _roll;
    float _yawCos, _yawSin;
    float *_x;			// states
    //double holdLat, holdLon;
    SRCDKF<AC_UKF> *kf;
    //navUkfStruct_t _navUkfData;
    runStruct_t _runData;

    void navUkfTimeUpdate(float *in, float *noise, float *out, float *u, float dt, int n);
    //static void altUkfPresUpdate(float *u, float *x, float *noise, float *y);
    void altDoPresUpdate();

	void navUkfInertialUpdate(float T);
	void simDoPresUpdate(float pres);
	void simDoAccUpdate(float accX, float accY, float accZ);
	void simDoMagUpdate(float magX, float magY, float magZ);

	void navUkfQuatExtractEuler(float *q, float *yaw, float *pitch, float *roll);
	void navUkfZeroRate(float zRate, int axis);
	void navUkfFinish(void);
	void navUkfRotateVectorByRevQuat(float *vr, float *v, float *q);
	void navUkfResetBias(void);
	void navUkfResetVels(void);
	void navUkfZeroPos(void);
	void navUkfZeroVel(void);
	void navUkfRotateVectorByQuat(float *vr, float *v, float *q);


	void navUkfNormalizeQuat(float *qr, float *q);
	void crossVector3(float *vr, float *va, float *vb);
	float dotVector3(float *va, float *vb);

	void navUkfRotateVecByMatrix(float *vr, float *v, float *m);
	void navUkfRotateVecByRevMatrix(float *vr, float *v, float *m);
	void navUkfQuatToMatrix(float *m, float *q, int normalize);
	void navUkfMatrixExtractEuler(float *m, float *yaw, float *pitch, float *roll);
	//void navUkfQuatExtractEuler(float *q, float *yaw, float *pitch, float *roll);
	void navUkfRotateQuat(float *qOut, float *qIn, float *rate);
	void navUkfRateUpdate(float *u, float *x, float *noise, float *y);
	void navUkfAccUpdate(float *u, float *x, float *noise, float *y);
	void navUkfMagUpdate(float *u, float *x, float *noise, float *y);
	void navUkfInitState(void);
	float compassNormalize(float heading);
	void navUkfNormalizeVec3(float *vr, float *v);

};


#endif /* AC_UKF_H_ */
