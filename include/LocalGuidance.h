#pragma once

#ifndef _LOCAL_GUIDANCE
#define _LOCAL_GUIDANCE

#include "Include.h"

void LidarDataRead();
void  lsr_MemWrite(void);

extern void CMD_delta_f();

extern double Lidar_dis[];
extern double Lidar_xCoor[];
extern double Lidar_yCoor[];
extern double delta_f;
extern int simul_cnt;

SOCKADDR_IN   servAddr, clntAddr;

typedef struct FIELD
{
	double xCoor[MAX_LIDAR_NUM];
	double xxCoor[MAX_LIDAR_NUM];
	double yCoor[MAX_LIDAR_NUM];
	double yyCoor[MAX_LIDAR_NUM];
	double dis[MAX_LIDAR_NUM];
	double ddis[MAX_LIDAR_NUM];
	double angle[MAX_LIDAR_NUM];
}FIELD;

double xxCoor[CLUSTER_NUM] = { 0.0, };
double yyCoor[CLUSTER_NUM] = { 0.0, };
double disCoor[CLUSTER_NUM] = { 0.0, };

FIELD OFF;
FIELD SFF;
FIELD IFF;
FIELD CLUSTER;

double sigSFF = 0.0;
double weightIFF = 0.0;
double max_angle = 0.0;
double k_local = 0.0;
//================================
// Buffer
double buf_dis[MAX_LIDAR_NUM] = { 0.0, };
double buf_xCoor[MAX_LIDAR_NUM] = { 0.0, };
double buf_xxCoor[CLUSTER_NUM] = { 0.0, };
double buf_yCoor[MAX_LIDAR_NUM] = { 0.0, };
double buf_yyCoor[CLUSTER_NUM] = { 0.0, };

//================================
// Parameter
int LidarRange = 0;

//================================
// Clustering
//
//double boundary = 0.1;
//int    minPoint = 10;
//
//
//int    count_Neighbors = 0;
//int    count_in_find = 0;
//int    count_distmean = 0;
//double dist_Xi_mean = 0.0;
//
//int    count_in_find_2 = 0;
//int    C = 0;
//int    k_in_expand = 0;
//int    j_in_expand = 0;
//int    count_Neighbors_2 = 0;
//int    count_order = 0;
//int    mixture = 0;
//int    super_k = 0;
//int    man_1 = 0;
//int    count_man_1 = 0;
//double man_1_x = 0.0;
//double man_1_y = 0.0;
//double sum_man_1_x = 0.0;
//double sum_man_1_y = 0.0;
//
//int    man_2 = 0;
//int    count_man_2 = 0;
//double sum_man_2_x = 0.0;
//double sum_man_2_y = 0.0;
//double man_2_x = 0.0;
//double man_2_y = 0.0;
//
//double  sorted_distmean[CLUSTER_NUM][2] = { 0, };
//double  Xi[CLUSTER_NUM][2] = { 0, };
//double  dist_sample[CLUSTER_NUM] = { 0, };
//double  buf_distmean[CLUSTER_NUM][2] = { 0, };
//int    IDX      [CLUSTER_NUM] = { 0.0, };
//double visited  [CLUSTER_NUM] = { 0.0, };
//double Neighbors[CLUSTER_NUM] = { 0.0, };
//double isnoise  [CLUSTER_NUM] = { 0.0, };
//double x_in_find[CLUSTER_NUM] = { 0.0, };
//int    super_out_find[MAX_NEIGHBOR_NUM] = { 0, };
//
//int    out_find[CLUSTER_NUM] = { 0, };
//int    out_find_2[CLUSTER_NUM] = { 0, };
#define MAX_LIDAR_NUM         761
#define MAX_LASER_POINT       761
#define minPoint       (int)    10
#define boundary       (double) 0.1


int    IDX[CLUSTER_NUM] = { 0.0, };

//////////////////////////////////////////////////////////////
double D[CLUSTER_NUM][CLUSTER_NUM] = { 0, 0 };
double value = 0.0;

double visited[CLUSTER_NUM] = { 0.0, };
double Neighbors[CLUSTER_NUM] = { 0.0, };
double isnoise[CLUSTER_NUM] = { 0.0, };
double x_in_find[CLUSTER_NUM] = { 0.0, };
double buf_deltaf_great[CLUSTER_NUM] = { 0.0, };
int    out_find[CLUSTER_NUM] = { 0, };
int    count_in_find = 0;
int    out_find_2[CLUSTER_NUM] = { 0, };
int    count_in_find_2 = 0;
int    super_out_find[520000] = { 0, };

int    count_Neighbors = 0;
int    count_Neighbors_2 = 0;
int      k_in_expand = 0;
int    j_in_expand = 0;
int    super_k = 0;
int      mixture = 0;
int    C = 0;
int first_count = 0;
int flag_cluster = 0;

//=========================================
//double  buf_xCoor[761] = { 0, };
//double  buf_yCoor[761] = { 0, };
double  Xi[CLUSTER_NUM][2] = { 0.0, };
double  dist_sample[CLUSTER_NUM] = { 0.0, };
double  buf_distmean[CLUSTER_NUM][2] = { 0.0, };
double  sorted_distmean[CLUSTER_NUM][2] = { 0.0, };

int flag_stop_kid = 0;
double dist_Xi_mean = 0.0;
//double deltaf = 0.0;
int    count_order = 0;

int    man_1 = 0;
double count_man_1 = 0;
double sum_man_1_x = 0.0;
double sum_man_1_y = 0.0;
double man_1_x = 0.0;
double man_1_y = 0.0;

int    man_2 = 0;
double count_man_2 = 0;
double sum_man_2_x = 0.0;
double sum_man_2_y = 0.0;
double man_2_x = 0.0;
double man_2_y = 0.0;

int    man_3 = 0;
double count_man_3 = 0;
double sum_man_3_x = 0.0;
double sum_man_3_y = 0.0;
double man_3_x = 0.0;
double man_3_y = 0.0;

int    man_4 = 0;
double count_man_4 = 0;
double sum_man_4_x = 0.0;
double sum_man_4_y = 0.0;
double man_4_x = 0.0;
double man_4_y = 0.0;

double XX;
double YY;

int count_distmean = 0;
int count_cncn = 0;


int C_IDX[100][CLUSTER_NUM] = { 0, };

double       bufOFF_rawdist[BUF_SIZE][MAX_LIDAR_NUM];
double       bufOFF_dist[BUF_SIZE][MAX_LIDAR_NUM];
double       bufOFF_angle[BUF_SIZE][MAX_LIDAR_NUM];
double       bufSFF_dist[BUF_SIZE][MAX_LIDAR_NUM];
double       bufSFF_angle[BUF_SIZE][MAX_LIDAR_NUM];
double       bufIFF_dist[BUF_SIZE][MAX_LIDAR_NUM];
double       bufIFF_angle[BUF_SIZE][MAX_LIDAR_NUM];


void dbdb(double* x, double* y);
void BIND_CLUSTER(void);
void main_data();
double(*mypdist2(double* x, double* y))[CLUSTER_NUM];
double Distance(double* x, double* y, int i, int j); // CLEAR







#endif // !_LOCAL_GUIDANCE