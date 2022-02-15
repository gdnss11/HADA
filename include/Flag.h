#pragma once
#ifndef FLAG_H
#define FLAG_H

#include "include.h"
#include "ExternVariable.h"

enum Mission_cmd {
	Do_Default_Command = 0,
	DO_Traffic_Command, DO_Left_Command, DO_Unpt_Command,
	
	DO_DeliveryA_Command,
	DO_DeliveryB_Command,
	FIN_DeliveryA_CMD,
	FIN_DeliveryB_CMD
};

int    idx_park = 0;
int    deliveryB_flag = 0;

int    idx_delcnt = 0;
//int    DelA_start_pt = 109;
//int    DelB_start_pt = 120; //정완학생 호출

int		STOP_LINE[20] = { 0 };
int     OBS_WP   [2] =  { 0 };
int     DEL_A_WP [20] = { 0 };
int     DEL_B_WP [20] = { 0 };
int		KID_LINE [2]  = { 20, 30 };

int     DELIVER_MOD = 0;
int     Delivery_flagA1 = 0;
int     Delivery_flagA2 = 0;
int     Delivery_flagA3 = 0;
int     Delivery_flagB1 = 0;
int     Delivery_flagB2 = 0;
int     Delivery_flagB3 = 0;
int     buf_delA_flag1[BUF_SIZE] = { 0 };
int     buf_delA_flag2[BUF_SIZE] = { 0 };
int     buf_delA_flag3[BUF_SIZE] = { 0 };
int     buf_delB_flag1[BUF_SIZE] = { 0 };
int     buf_delB_flag2[BUF_SIZE] = { 0 };
int     buf_delB_flag3[BUF_SIZE] = { 0 };
int		Traffic_flag = 0;
int     flagA_cnt = 0;
int     flagB_cnt = 0;
int     Fin_cnt   = 0;
int     delA_stopFlag = 0;
int     delB_stopFlag = 0;
int		ERP_flag = 0;

int      mission_park, mission_obs, 
mission_sign, mission_del = 0;

int      UNPT_LINE = 0;   //비보호
int      LEFT_LINE = 0;

double init_Pos_lat_DA = 0.0;
double init_Pos_lon_DA = 0.0;
double delA_Vec_BodyToWay[1][2] = { 0.0, 0.0 };
double delA_Dist_R = 0.0;

double init_Pos_lat_DB = 0.0;
double init_Pos_lon_DB = 0.0;
double delB_Vec_BodyToWay[1][2] = { 0.0, 0.0 };
double delB_Dist_R = 0.0;
#endif