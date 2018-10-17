#ifndef _srcdkf_h
#define _srcdkf_h

#include "aq_math.h"
#include "math.h"

#include <stdio.h>
#include <string.h>
/*#ifdef	__cplusplus
extern "C"
{
#endif*/

#define SRCDKF_H        (__sqrtf(3.0f) * 3.0f)//(__sqrtf(3.0f) * 3.0f)
#define SRCDKF_RM       0.0001f         // Robbins-Monro stochastic term

#ifndef MAX
#define MAX(a, b)       ((a > b) ? a : b)//从.h文件中移过来的
#endif

template <typename T>
class SRCDKF{
public:


	SRCDKF(T *pInstance){
		_pInstance = pInstance;
		//kf = (srcdkf_t *)aqDataCalloc(1, sizeof(srcdkf_t));
	}

	~SRCDKF()
	{
		if(_Sx.pData){
			_Sx.pData = NULL;
		}
		if(_SxT.pData){
			_SxT.pData = NULL;
		}
		if(_Sv.pData){
			_Sv.pData = NULL;
		}
		if(_Sn.pData){
			_Sn.pData = NULL;
		}
		if(_x.pData){
			_x.pData = NULL;
		}
		if(_Xa.pData){
			_Xa.pData = NULL;
		}
		if(_qrTempS.pData){
			_qrTempS.pData = NULL;
		}
		if(_y.pData){
			_y.pData = NULL;
		}
		if(_Y.pData){
			_Y.pData = NULL;
		}
		if(_qrTempM.pData){
			_qrTempM.pData = NULL;
		}
		if(_Sy.pData){
			_Sy.pData = NULL;
		}
		if(_SyT.pData){
			_SyT.pData = NULL;
		}
		if(_SyC.pData){
			_SyC.pData = NULL;
		}
		if(_Pxy.pData){
			_Pxy.pData = NULL;
		}
		if(_C1.pData){
			_C1.pData = NULL;
		}
		if(_C1T.pData){
			_C1T.pData = NULL;
		}
		if(_C2.pData){
			_C2.pData = NULL;
		}
		if(_D.pData){
			_D.pData = NULL;
		}
		if(_K.pData){
			_K.pData = NULL;
		}
		if(_inov.pData){
			_inov.pData = NULL;
		}
		if(_xUpdate.pData){
			_xUpdate.pData = NULL;
		}
		if(_qrFinal.pData){
			_qrFinal.pData = NULL;
		}
		if(_Q.pData){
			_Q.pData = NULL;
		}
		if(_R.pData){
			_R.pData = NULL;
		}
		if(_AQ.pData){
			_AQ.pData = NULL;
		}


		if(_xOut){
			_xOut = NULL;
		}
		if(_xNoise){
			_xNoise = NULL;
		}
		if(_xIn){
			_xIn = NULL;
		}

	}

	typedef void (T::*SRCDKFTimeUpdate_t)(float32_t *x_I, float32_t *noise_I, float32_t *x_O, float32_t *u, float32_t dt, int n);//定义了一个函数
	typedef void (T::*SRCDKFMeasurementUpdate_t)(float32_t *u, float32_t *x, float32_t *noise_I, float32_t *y);

	void setTimeUpdate(SRCDKFTimeUpdate_t pTimeUpdate){
		_timeUpdate = pTimeUpdate;
	}
	void setMeasurementUpdate(SRCDKFMeasurementUpdate_t pMeasurementUpdate){
		_pMeasurementUpdate = pMeasurementUpdate;
	}


	void srcdkfInit(int s, int m, int v, int n)
	{
		//srcdkf_t *f;
		int maxN = MAX(v, n);

		//f = (srcdkf_t *)aqDataCalloc(1, sizeof(srcdkf_t));

		_S = s;
		_V = v;

		matrixInit(&_Sx, s, s);
		matrixInit(&_SxT, s, s);
		matrixInit(&_Sv, v, v);
		matrixInit(&_Sn, n, n);
		matrixInit(&_x, s, 1);
		matrixInit(&_Xa, s+maxN, 1+(s+maxN)*2);

		matrixInit(&_qrTempS, s, (s+v)*2);
		matrixInit(&_y, m, 1);
		matrixInit(&_Y, m, 1+(s+n)*2);
		matrixInit(&_qrTempM, m, (s+n)*2);
		matrixInit(&_Sy, m, m);
		matrixInit(&_SyT, m, m);
		matrixInit(&_SyC, m, m);
		matrixInit(&_Pxy, s, m);
		matrixInit(&_C1, m, s);
		matrixInit(&_C1T, s, m);
		matrixInit(&_C2, m, n);
		matrixInit(&_D, m, s+n);
		matrixInit(&_K, s, m);
		matrixInit(&_inov, m, 1);
		matrixInit(&_xUpdate, s, 1);
		matrixInit(&_qrFinal, s, 2*s + 2*n);
		matrixInit(&_Q, s, s+n);	// scratch
		matrixInit(&_R, n, n);	// scratch
		matrixInit(&_AQ, s, n);	// scratch

		_xOut = (float32_t *)aqDataCalloc(s, sizeof(float32_t));
		_xNoise = (float32_t *)aqDataCalloc(maxN, sizeof(float32_t));
		_xIn = (float32_t *)aqDataCalloc(s, sizeof(float32_t));

		_h = SRCDKF_H;
		_hh = _h*_h;
	//	f->w0m = (f->hh - (float32_t)s) / f->hh;	// calculated in process
		_wim = 1.0f / (2.0f * _hh);
		_wic1 = __sqrtf(1.0f / (4.0f * _hh));
		_wic2 = __sqrtf((_hh - 1.0f) / (4.0f * _hh*_hh));

		//_timeUpdate = timeUpdate;

		// return f;
	}

	float *srcdkfGetState() {
	    return _x.pData;
	}

	void srcdkfSetVariance(float32_t *q, float32_t *v, float32_t *n, int nn) {
		float32_t *Sx = _Sx.pData;
		float32_t *Sv = _Sv.pData;
		float32_t *Sn = _Sn.pData;
		int i;

		// state variance
		if (q)
			for (i = 0; i < _S; i++)
				Sx[i*_S + i] = __sqrtf(fabsf(q[i]));

		// process noise
		if (v)
			for (i = 0; i < _V; i++)
				Sv[i*_V + i] = __sqrtf(fabsf(v[i]));

		// observation noise
		if (n && nn) {
			// resize Sn
			_Sn.numRows = nn;
			_Sn.numCols = nn;

			for (i = 0; i < nn; i++)
				Sn[i*nn + i] = __sqrtf(fabsf(n[i]));
		}
	}

	void srcdkfGetVariance(float32_t *q) {
		float32_t *SxData = _Sx.pData;
		int i;

		// state variance
		if (q)
			for (i = 0; i < _S; i++) {
				q[i] = SxData[i*_S + i];
				q[i] = q[i]*q[i];
			}
	}

	void srcdkfTimeUpdate(float32_t *u, float32_t dt) {
		int S = _S;			// number of states
		int V = _V;			// number of noise variables
		int L;				// number of sigma points
		float32_t *x = _x.pData;	// state estimate
		float32_t *Xa = _Xa.pData;	// augmented sigma points

		float32_t *qrTempS = _qrTempS.pData;
		int i, j;

		srcdkfCalcSigmaPoints(&_Sv);
		L = _L;

		//printf("timeUpdate\n");
		(_pInstance->*_timeUpdate)(&Xa[0], &Xa[S*L], &Xa[0], u, dt, L);

		// sum weighted resultant sigma points to create estimated state
		_w0m = (_hh - (float32_t)(S+V)) / _hh;
		for (i = 0; i < S; i++) {
			int rOffset = i*L;

			x[i] = Xa[rOffset + 0] * _w0m;

			for (j = 1; j < L; j++)
				x[i] += Xa[rOffset + j] * _wim;
		}

		// update state covariance
		for (i = 0; i < S; i++) {
			int rOffset = i*(S+V)*2;

			for (j = 0; j < S+V; j++) {
				qrTempS[rOffset + j] = (Xa[i*L + j + 1] - Xa[i*L + S+V + j + 1]) * _wic1;
				qrTempS[rOffset + S+V + j] = (Xa[i*L + j + 1] + Xa[i*L + S+V + j + 1] - 2.0f*Xa[i*L + 0]) * _wic2;
			}
		}

		qrDecompositionT_f32(&_qrTempS, NULL, &_SxT);   // with transposition
		arm_mat_trans_f32(&_SxT, &_Sx);
	}

	void srcdkfMeasurementUpdate(float32_t *u, float32_t *ym, int M, int N, float32_t *noise/*, SRCDKFMeasurementUpdate_t *measurementUpdate*/) {
		int S = _S;				// number of states
		float32_t *Xa = _Xa.pData;			// sigma points
		float32_t *xIn = _xIn;			// callback buffer
		float32_t *xNoise = _xNoise;		// callback buffer
		float32_t *xOut = _xOut;			// callback buffer
		float32_t *Y = _Y.pData;			// measurements from sigma points
		float32_t *y = _y.pData;			// measurement estimate
		float32_t *Sn = _Sn.pData;			// observation noise covariance
		float32_t *qrTempM = _qrTempM.pData;
		float32_t *C1 = _C1.pData;
		float32_t *C1T = _C1T.pData;
		float32_t *C2 = _C2.pData;
		float32_t *D = _D.pData;
		float32_t *inov = _inov.pData;		// M x 1 matrix
		float32_t *xUpdate = _xUpdate.pData;	// S x 1 matrix
		float32_t *x = _x.pData;			// state estimate
		float32_t *Sx = _Sx.pData;
		float32_t *Q = _Q.pData;
		float32_t *qrFinal = _qrFinal.pData;
		int L;					// number of sigma points
		int i, j;

		// make measurement noise matrix if provided
		if (noise) {
			_Sn.numRows = N;
			_Sn.numCols = N;
			//arm_fill_f32(0.0f, f->Sn.pData, N*N);
			memset(_Sn.pData, 0, sizeof(float32_t)*N*N);
			for (i = 0; i < N; i++)
				//__sqrtf(fabsf(noise[i]), &Sn[i*N + i]);
				arm_sqrt_f32(fabsf(noise[i]), &Sn[i*N + i]);
		}

		// generate sigma points
		srcdkfCalcSigmaPoints(&_Sn);
		L = _L;

		// resize all N and M based storage as they can change each iteration
		_y.numRows = M;
		_Y.numRows = M;
		_Y.numCols = L;
		_qrTempM.numRows = M;
		_qrTempM.numCols = (S+N)*2;
		_Sy.numRows = M;
		_Sy.numCols = M;
		_SyT.numRows = M;
		_SyT.numCols = M;
		_SyC.numRows = M;
		_SyC.numCols = M;
		_Pxy.numCols = M;
		_C1.numRows = M;
		_C1T.numCols = M;
		_C2.numRows = M;
		_C2.numCols = N;
		_D.numRows = M;
		_D.numCols = S+N;
		_K.numCols = M;
		_inov.numRows = M;
		_qrFinal.numCols = 2*S + 2*N;

		// Y = h(Xa, Xn)
		for (i = 0; i < L; i++) {
			for (j = 0; j < S; j++)
				xIn[j] = Xa[j*L + i];

			for (j = 0; j < N; j++)
				xNoise[j] = Xa[(S+j)*L + i];

			//measurementUpdate(u, xIn, xNoise, xOut);
			(_pInstance->*_pMeasurementUpdate)(u, xIn, xNoise, xOut);

			for (j = 0; j < M; j++)
				Y[j*L + i] = xOut[j];
		}

		// sum weighted resultant sigma points to create estimated measurement
		_w0m = (_hh - (float32_t)(S+N)) / _hh;
		for (i = 0; i < M; i++) {
			int rOffset = i*L;

			y[i] = Y[rOffset + 0] * _w0m;

			for (j = 1; j < L; j++)
				y[i] += Y[rOffset + j] * _wim;
		}

		// calculate measurement covariance components
		for (i = 0; i < M; i++) {
			int rOffset = i*(S+N)*2;

			for (j = 0; j < S+N; j++) {
				float32_t c, d;

				c = (Y[i*L + j + 1] - Y[i*L + S+N + j + 1]) * _wic1;
				d = (Y[i*L + j + 1] + Y[i*L + S+N + j + 1] - 2.0f*Y[i*L]) * _wic2;

				qrTempM[rOffset + j] = c;
				qrTempM[rOffset + S+N + j] = d;

				// save fragments for future operations
				if (j < S) {
					C1[i*S + j] = c;
					C1T[j*M + i] = c;
				}
				else {
					C2[i*N + (j-S)] = c;
				}
				D[i*(S+N) + j] = d;
			}
		}

		qrDecompositionT_f32(&_qrTempM, NULL, &_SyT);	// with transposition

		arm_mat_trans_f32(&_SyT, &_Sy);
		arm_mat_trans_f32(&_SyT, &_SyC);		// make copy as later Div is destructive

		// create Pxy
		arm_mat_mult_f32(&_Sx, &_C1T, &_Pxy);

		// K = (Pxy / SyT) / Sy
		matrixDiv_f32(&_K, &_Pxy, &_SyT, &_Q, &_R, &_AQ);
		matrixDiv_f32(&_K, &_K, &_Sy, &_Q, &_R, &_AQ);

		// x = x + k(ym - y)
		for (i = 0; i < M; i++)
			inov[i] = ym[i] - y[i];
		arm_mat_mult_f32(&_K, &_inov, &_xUpdate);

		for (i = 0; i < S; i++)
			x[i] += xUpdate[i];

		// build final QR matrix
		//	rows = s
		//	cols = s + n + s + n
		//	use Q as temporary result storage

		_Q.numRows = S;
		_Q.numCols = S;
		arm_mat_mult_f32(&_K, &_C1, &_Q);
		for (i = 0; i < S; i++) {
			int rOffset = i*(2*S + 2*N);

			for (j = 0; j < S; j++)
				qrFinal[rOffset + j] = Sx[i*S + j] - Q[i*S + j];
		}

		_Q.numRows = S;
		_Q.numCols = N;
		arm_mat_mult_f32(&_K, &_C2, &_Q);
		for (i = 0; i < S; i++) {
			int rOffset = i*(2*S + 2*N);

			for (j = 0; j < N; j++)
				qrFinal[rOffset + S+j] = Q[i*N + j];
		}

		_Q.numRows = S;
		_Q.numCols = S+N;
		arm_mat_mult_f32(&_K, &_D, &_Q);
		for (i = 0; i < S; i++) {
			int rOffset = i*(2*S + 2*N);

			for (j = 0; j < S+N; j++)
				qrFinal[rOffset + S+N+j] = Q[i*(S+N) + j];
		}

		// Sx = qr([Sx-K*C1 K*C2 K*D]')
		// this method is not susceptable to numeric instability like the Cholesky is
		qrDecompositionT_f32(&_qrFinal, NULL, &_SxT);	// with transposition
		arm_mat_trans_f32(&_SxT, &_Sx);
	}
	//srcdkf_t *paramsrcdkfInit(int w, int d, int n, SRCDKFMeasurementUpdate_t *map);
	//void paramsrcdkfUpdate(float32_t *u, float32_t *d);
	void paramsrcdkfSetVariance(float32_t *v, float32_t *n) {
		float32_t *rDiag = _rDiag.pData;
		int i;

		srcdkfSetVariance(v, v, n, _N);

		for (i = 0; i < _S; i++)
			rDiag[i] = 0.0f;
	}

	void paramsrcdkfGetVariance(float32_t *v, float32_t *n) {
		float32_t *Sx = _Sx.pData;
		float32_t *Sn = _Sn.pData;
		int i;

		// artificial parameter variance
		if (v)
			for (i = 0; i < _S; i++) {
				v[i] = Sx[i*_S + i];
				v[i] = v[i]*v[i];
			}

		if (n)
			for (i = 0; i < _N; i++) {
				n[i] = Sn[i*_N + i];
				n[i] = n[i]*n[i];
			}
	}

	void paramsrcdkfSetRM(float32_t rm) {
		_rm = rm;
	}

private:
	//srcdkf_t *kf;

    int _S;
    int _V;
    int _M;          // only used for parameter estimation
    int _N;          // only used for parameter estimation
    int _L;

    float32_t _h;
    float32_t _hh;
    float32_t _w0m, _wim, _wic1, _wic2;
    float32_t _rm;

    arm_matrix_instance_f32 _Sx;     // state covariance状态协方差
    arm_matrix_instance_f32 _SxT;    // Sx transposed状态协方差的转置
    arm_matrix_instance_f32 _Sv;     // process noise过程噪声
    arm_matrix_instance_f32 _Sn;     // observation noise观测噪声
    arm_matrix_instance_f32 _x;      // state estimate vector状态估计向量
    arm_matrix_instance_f32 _Xa;     // augmented sigma points采样点
    float32_t *_xIn;
    float32_t *_xNoise;
    float32_t *_xOut;
    arm_matrix_instance_f32 _qrTempS;//状态协方差
    arm_matrix_instance_f32 _Y;      // resultant measurements from sigma points
    arm_matrix_instance_f32 _y;      // measurement estimate vector
    arm_matrix_instance_f32 _qrTempM;
    arm_matrix_instance_f32 _Sy;     // measurement covariance测量协方差
    arm_matrix_instance_f32 _SyT;    // Sy transposed
    arm_matrix_instance_f32 _SyC;    // copy of Sy
    arm_matrix_instance_f32 _Pxy;
    arm_matrix_instance_f32 _C1;
    arm_matrix_instance_f32 _C1T;
    arm_matrix_instance_f32 _C2;
    arm_matrix_instance_f32 _D;
    arm_matrix_instance_f32 _K;      //卡尔曼滤波增益
    arm_matrix_instance_f32 _KT;     // only used for param est
    arm_matrix_instance_f32 _inov;   // inovation
    arm_matrix_instance_f32 _inovT;// only used for param est
    arm_matrix_instance_f32 _xUpdate;
    arm_matrix_instance_f32 _qrFinal;
    arm_matrix_instance_f32 _rDiag;
    arm_matrix_instance_f32 _Q, _R, _AQ;       // scratch

    SRCDKFTimeUpdate_t _timeUpdate;//定义了一个函数类型的指针
    //SRCDKFMeasurementUpdate_t *_map; // only used for param est
    SRCDKFMeasurementUpdate_t _pMeasurementUpdate;

    T *_pInstance;

    void srcdkfCalcSigmaPoints(arm_matrix_instance_f32 *Sn) {
    	int S = _S;			// number of states
    	int N = Sn->numRows;		// number of noise variables
    	int A = S+N;			// number of agumented states
    	int L = 1+A*2;			// number of sigma points
    	float32_t *x = _x.pData;	// state
    	float32_t *Sx = _Sx.pData;	// state covariance
    	float32_t *Xa = _Xa.pData;	// augmented sigma points
    	int i, j;

    	// set the number of sigma points
    	_L = L;

    	// resize output matrix
    	_Xa.numRows = A;
    	_Xa.numCols = L;

    	//	-	   -
    	// Sa =	| Sx	0  |
    	//	| 0	Sn |
    	//	-	   -
    	// xa = [ x 	0  ]
    	// Xa = [ xa  (xa + h*Sa)  (xa - h*Sa) ]
    	//
    	for (i = 0; i < A; i++) {
    		int rOffset = i*L;
    		float32_t base = (i < S) ? x[i] : 0.0f;

    		Xa[rOffset + 0] = base;

    		for (j = 1; j <= A; j++) {
    			float32_t t = 0.0f;

    			if (i < S && j < S+1)
    				t = Sx[i*S + (j-1)]*_h;

    			if (i >= S && j >= S+1)
    				t = Sn->pData[(i-S)*N + (j-S-1)]*_h;

    			Xa[rOffset + j]     = base + t;
    			Xa[rOffset + j + A] = base - t;
    		}
    	}
    }
};

/*extern srcdkf_t *srcdkfInit(int s, int m, int v, int n, SRCDKFTimeUpdate_t *timeUpdate);
extern float *srcdkfGetState(srcdkf_t *f);
extern void srcdkfSetVariance(srcdkf_t *f, float32_t *q, float32_t *v, float32_t *n, int nn);
extern void srcdkfGetVariance(srcdkf_t *f, float32_t *q);
extern void srcdkfTimeUpdate(srcdkf_t *f, float32_t *u, float32_t dt);
extern void srcdkfMeasurementUpdate(srcdkf_t *f, float32_t *u, float32_t *y, int M, int N, float32_t *noise, SRCDKFMeasurementUpdate_t *measurementUpdate);
extern void srcdkfFree(srcdkf_t *f);
extern srcdkf_t *paramsrcdkfInit(int w, int d, int n, SRCDKFMeasurementUpdate_t *map);
extern void paramsrcdkfUpdate(srcdkf_t *f, float32_t *u, float32_t *d);
extern void paramsrcdkfSetVariance(srcdkf_t *f, float32_t *v, float32_t *n);
extern void paramsrcdkfGetVariance(srcdkf_t *f, float32_t *v, float32_t *n);
extern void paramsrcdkfSetRM(srcdkf_t *f, float32_t rm);*/

/*#ifdef  __cplusplus
}
#endif*/  /* end of __cplusplus */

#endif
