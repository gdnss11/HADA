#pragma once
#ifndef PARKING_H
#define PARKING_H

#include "Include.h"



int PARKINGMODE = PARKING_INIT;
//int PARKINGMODE = STOP_TO_GO;

double CAR_X = 0.0;
double CAR_Y = 0.0;
double CAR_ATT = 0.0;

//double PARK_X = 129.3869603 * LON2METER;
//double PARK_Y = 36.1041768 * LAT2METER;
//double PARK_ATT = (90 + 71.29) * DEG2RAD;   //  (90.0 - YAW) * DEG2RAD   //[GAMMA]  

double PARK_X = 0.0;
double PARK_Y = 0.0;
double PARK_ATT = 0.0;   
int    CHOOSE_PARKING_IDX = 0;

int    park0[4] = { 13,17,20,21 };
//===============================
// Parking Mission Car Initialization
double parkInit_X = 0.0;
double parkInit_Y = 0.0;
double parkInit_ATT = 0.0;

double ParkInit_X_SUM = 0.0;
double ParkInit_Y_SUM = 0.0;
double ParkInit_ATT_SUM = 0.0;

double PARK_INIT_X = 0.0;
double PARK_INIT_Y = 0.0;
double PARK_INIT_ATT = 0.0;

int ParkInit_idx = 0;
////
//double buf_ParkInitX[7] = { 129.3862128, 129.3861952, 129.3861845, 129.3861605, 129.3860684, 129.38605614, 129.3860481 };
//double buf_ParkInitY[7] = {36.1041133, 36.104082, 36.10405075, 36.1039896, 36.10377602, 36.10375647, 36.103736};
//double buf_ParkInitAtt[7] = {-113.5, -113.5, -113.5, -113.5, -70.35, -70.35, -70.35};  // HANDONG





double buf_ParkInitX[13]   = { 126.7731242035,  126.7731474975,  126.7731633895,126.773203623,126.7732155745,126.773231191,126.773246929,126.773291599,126.773304961,126.773321124,126.7733378005,126.773356221,126.773371811 };
double buf_ParkInitY[13]   = { 37.2390809,      37.239110426,    37.239139577,  37.239198717, 37.239215012,  37.239236307, 37.239256784, 37.2393306325,37.2393520015,37.2393759,   37.2393991045, 37.23942326,  37.239446119 };
double buf_ParkInitAtt[13] = { 69.57,           69.57,           69.57,         117.59,       117.59,        117.59,       117.59,       88.01,        88.01,        88.01,        88.01,         88.01,        88.01 };


//===============================
// Trajectory Generation
double xInc = 0.0;
double yInc = 0.0;
double CAR_X0 = 0.0;
double CAR_Y0 = 0.0;
double CAR_ATT0 = 0.0;
double X_ParkingWP = 0.0;
double Y_ParkingWP = 0.0;
double X_Star = 0.0;
double Y_Star = 0.0;

int star_WP_idx = 0;

double buf_X_ParkingWP[BUF_SIZE] = { 0.0, };
double buf_Y_ParkingWP[BUF_SIZE] = { 0.0, };
double buf_ATT_ParkingWP[BUF_SIZE] = { 0.0, };

int parkWP_idx = 0;
int parkNwp_idx = 0;

double SLOPE_init = 0.0;
double SLOPE_goal = 0.0;

double intY_init = 0.0;
double intY_goal = 0.0;
double OSET_init = 0.0;
double OSET_goal = 0.0;

double intY_init_1 = 0.0;
double intY_init_2 = 0.0;
double intY_goal_1 = 0.0;
double intY_goal_2 = 0.0;

double X_1 = 0.0;
double Y_1 = 0.0;

double X_2 = 0.0;
double Y_2 = 0.0;

double X_3 = 0.0;
double Y_3 = 0.0;

double X_4 = 0.0;
double Y_4 = 0.0;

double X_mean = 0.0;
double Y_mean = 0.0;

double DIS_1 = 0.0;
double DIS_2 = 0.0;
double DIS_3 = 0.0;
double DIS_4 = 0.0;
double DIS_CUR = 0.0;

double X_one = 0.0;
double Y_one = 0.0;

double angle_delta = 0.0;

//===============================
// Kanayama Controller
int kana_idx = 0;
double Dist_XtoWP = 0.0;
double Dist_YtoWP = 0.0;
double parkDist = 0.0;
double Kana_angle_error = 0.0;

double buf_parking_psi[BUF_SIZE] = { 0.0, };
double buf_parking_w_cmd[BUF_SIZE] = { 0.0, };
double buf_parking_delta_des[BUF_SIZE] = { 0.0, };
double buf_parking_Xcarpath[BUF_SIZE] = { 0.0, };
double buf_parking_Ycarpath[BUF_SIZE] = { 0.0, };
double buf_parkDist[BUF_SIZE] = { 0.0, };
double buf_parking_delta_f[BUF_SIZE] = { 0.0, };
double buf_parking_vel_cmd[BUF_SIZE] = { 0.0, };
double buf_Star_XY[BUF_SIZE] = { 0.0, };
double buf_Kana_Angle_Error[BUF_SIZE] = { 0.0, };

double Pc[3] = { 0.0, };
double Pr[3] = { 0.0, };
double Te[3][3] = { 0.0, };
double Pe[3] = { 0.0, };

double gain_delta_f = 0.0;
double MIN_PARKDIS = 0.0;

double Parking_Vel_cmd = 0.0;
double Parking_w_cmd = 0.0;
double Parking_delta_des = 0.0;

//===============================
// Stopping in the parking space
int parking_stop_idx = 0;

//===============================
// Backingout of parking space
double buf_backX[BUF_SIZE] = { 0.0, };
double buf_backY[BUF_SIZE] = { 0.0, };
double Back_DistX = 0.0;
double Back_DistY = 0.0;
double Back_Dist = 0.0;

int park_idx = 0;
int back_idx = 0;
int stop_idx = 0;

// gear 바꾸기 to 후진
int vehicle_gear = 0;


#endif // !MISSION_HEAD
