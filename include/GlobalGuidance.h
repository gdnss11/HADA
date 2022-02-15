#pragma once
#ifndef _GLOBAL_GUIDANCE
#define _GLOBAL_GUIDANCE

#include "include.h"

void Save_MIDwaypoint();
void Select_Xsens_yaw();
//===========================================
// Buffer
double GPS_waypoint_Lat[BUF_SIZE] = { 0, };
double GPS_waypoint_Lon[BUF_SIZE] = { 0, };
int    idx_max         [BUF_SIZE] = { 0, };
int    buf_Index       [BUF_SIZE] = { 0, };
int    buf_Index2      [BUF_SIZE] = { 0, };
double buf_delta       [BUF_SIZE] = { 0, };
double buf_lambda      [BUF_SIZE] = { 0, };
double buf_Dist_R      [BUF_SIZE] = { 0, };
double buf_gamma       [BUF_SIZE] = { 0, };
double buf_delta_f     [BUF_SIZE] = { 0, };
double buf_waylatnow   [BUF_SIZE] = { 0, };
double buf_waylonnow   [BUF_SIZE] = { 0, };

//===========================================
// GPS & IMU
double Real_Yaw  = 0.0;

double global_minD = 0.0;
double  global_K = 0.0;

int    cnt_initial = 0;
double Lat_init    = 0.0;
double Lon_init    = 0.0;
double LAT0        = 0.0;
double LON0        = 0.0;

double GPS_waypoint_Lat_now = 0.0;
double GPS_waypoint_Lon_now = 0.0;

double  Waypoint_lat_cal = 0.0;
double  Waypoint_lon_cal = 0.0;


int DeliveryIDX[2][2] = { 0.0, };
//===========================================
// Pursuit Guidance
double Dist_R  = 0.0;
double lambda  = 0.0;
double gamma   = 0.0;
double delta   = 0.0;
double delta_f = 0.0;
int    idx_way = 0;
int    vel_cmd = 0;
int    brk_cmd = 0;

//===========================================
// Calculation
double Vec_BodyToWay[1][2] = { 0.0, 0.0 };


//===========================================
int    init_detect                  = 0    ;
int    final_detect                 = 0    ;
int    Start_point                  = 0    ;
int    idx_sign          [BUF_SIZE] = { 0 };
int    idx_obs           [2 ]       = { 0 };
int    idx_del           [2 ]       = { 0 };
int    Sign_IDXpoint     [10]       = { 0 };
int    waypt_IDX		            = { 0 };
int    xsens_idx         [BUF_SIZE] = { 0.0, };
double xsens_lat         [BUF_SIZE] = { 0.0, };
double xsens_lon         [BUF_SIZE] = { 0.0, };
double initialization_yaw[BUF_SIZE] = { 0.0, };
double Way_dist                     = 0.0  ;
double Min_dist                     = 100.0  ;


#endif //!_GLOBAL_GUIDANCE