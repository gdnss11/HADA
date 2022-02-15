#pragma once

#ifndef _EKF
#define _EKF

#include "Include.h"

void variable();
void INVERSE_2X2(double _MAT[2][2], double _MAT_INV[2][2]);
void DCM_change(double _MAT[3][3], double _roll, double _pitch, double _yaw);
void System_update();


//======================================
// Design Variable
//======================================
#define R0      6371000.0
#define R_COV   (double) (0.05) //[m]
#define We      0.0000729
#define POPOS   1.0
#define POVEL   1.0
#define SIGQ    0.001

double sigPOS_LAT = 0.0;
double sigPOS_LON = 0.0;

double lat_filtered = 0.0;
double lon_filtered = 0.0;

double eX_LAT = 0.0;
double eX_LON = 0.0;
double eX_VN = 0.0;
double eX_VE = 0.0;
double FN = 0.0;
double FE = 0.0;


//======================================
// Constant marix
//======================================
double MAT_R[2][2] = { 0.0, };
double eX_[4][1] = { 0.0, };
double pX_[4][1] = { 0.0, };
double MAT_F[4][4] = { 0.0, };
double MAT_F_T[4][4] = { 0.0, };
double RESIDUAL[2][1] = { 0.0, };

double F_eP[4][4] = { 0.0, };
double F_eP_FT[4][4] = { 0.0, };

double pP[4][4] = { 0.0, };
double pY[2][1] = { 0.0, };
double pP_HT[4][2] = { 0.0, };
double H_pP[2][4] = { 0.0, };
double H_pP_HT[2][2] = { 0.0, };
double H_pP_HT_R[2][2] = { 0.0, };
double INV_H_pP_HT_R[2][2] = { 0.0, };
double MAT_K[4][2] = { 0.0, };

double K_H[4][4] = { 0.0, };
double EYE_K_H[4][4] = { 0.0, };

double K_RESIDUAL[4][1] = { 0.0, };

double ACC_BDY[3][1] = { 0.0, };
double ACC_NED[3][1] = { 0.0, };
double MAT_DCM[3][3] = { 0.0, };

double POS_GPS[2][1] = { 0.0, };

double eP[4][4] = {
	{ POPOS, 0.0  , 0.0  , 0.0  },
	{ 0.0  , POVEL, 0.0  , 0.0  },
	{ 0.0  , 0.0  , POPOS, 0.0  },
	{ 0.0  , 0.0  , 0.0  , POVEL},
};

double MAT_Q[4][4] = {
	{ QUAD(SAMPLING_TIME) / 4.0 * SQUA(SIGQ), SQUA(SAMPLING_TIME) / 2.0 * SQUA(SIGQ),                                    0.0,                                    0.0},
	{ SQUA(SAMPLING_TIME) / 2.0 * SQUA(SIGQ),             SAMPLING_TIME * SQUA(SIGQ),                                    0.0,                                    0.0},
	{                                    0.0,                                    0.0, QUAD(SAMPLING_TIME) / 4.0 * SQUA(SIGQ), SQUA(SAMPLING_TIME) / 2.0 * SQUA(SIGQ)},
	{                                    0.0,                                    0.0, SQUA(SAMPLING_TIME) / 2.0 * SQUA(SIGQ),             SAMPLING_TIME * SQUA(SIGQ)}
};

double MAT_H[2][4] = {
	{ 1.0, 0.0, 0.0, 0.0},
	{ 0.0, 0.0, 1.0, 0.0}
};

double MAT_H_T[4][2] = {
	{ 1.0, 0.0},
	{ 0.0, 0.0},
	{ 0.0, 1.0},
	{ 0.0, 0.0}
};

double EYE[4][4] = {
	{ 1.0, 0.0, 0.0, 0.0 },
	{ 0.0, 1.0, 0.0, 0.0 },
	{ 0.0, 0.0, 1.0, 0.0 },
	{ 0.0, 0.0, 0.0, 1.0 },
};


double buf_Filtered_Lat[BUF_SIZE] = { 0, };
double buf_Filtered_Lon[BUF_SIZE] = { 0, };
double buf_RES_LAT[BUF_SIZE] = { 0, };
double buf_RES_LON[BUF_SIZE] = { 0, };


#endif // !_EKF
