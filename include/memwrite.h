#ifndef _DATA_WRITE
#define _DATA_WRITE

#include "Include.h"
void OpenClientXsens(void);
void OpenClientNovA_GPS();
void OpenClientNovA_VEL(void);
void OpenClientLidar(void);
//======================================
// Data Received
//======================================
double  roll = 0.0, pitch = 0.0, yaw = 0.0;
double  accX = 0.0, accY = 0.0, accZ = 0.0;
double  latitude = 0.0;
double  longitude = 0.0;
double  direction = 0.0;
double  velocity = 0.0;

int     index_gps = 0;
int     fixqual;
double  yaw_OFFSET = 0.0;
double  yaw_buffer = 0.0;

double  Lidar_dis  [MAX_LIDAR_NUM];
double  Lidar_xCoor[MAX_LIDAR_NUM];
double  Lidar_yCoor[MAX_LIDAR_NUM];

//======================================
// Strufixqual  cture
//======================================
typedef struct
{
	double   xCoordinate[MAX_LIDAR_NUM];
	double   yCoordinate[MAX_LIDAR_NUM];
	double         dist [MAX_LIDAR_NUM];
	double        angle [MAX_LIDAR_NUM];
	char     stringDATA [MAX_LASER_RECEIVE];
}Lidar_smdat;

typedef struct
{
	int		SIM_count;
	double  SIM_Time;
	double  euler_x;	  //roll 
	double  euler_y;	  //pitch
	double  euler_z;	  //yaw 
	double  acc_x;		  //accX
	double  acc_y;		  //accY
	double  acc_z;		  //accZ
	double  vel_x;
	double  vel_y;
	double  vel_z;
	double  rot_x;
	double  rot_y;
	double  rot_z;
}Xsens_smdat;

typedef struct
{
	//double  BestLat;
	//double  BestLon;
	int		SIM_count;
	double  SIM_Time;
	long	UTChour;
	long	UTCmin;
	long	UTCsec;
	long	UTCmsec;
	int		LatDegree;
	double	LatMinute;
	char	NS;
	int		LongDegree;
	double	LongMinute;
	char	EW;
	char	FixQual;
	int		N_Sat;
	double   DOP;
	int		flag;

	double  Direction;
	double  Velocity;
	//double  BestLat;
	//double  BestLon;
}NovATel_smdat;


HANDLE           hMampF_CLI_Lidar;
HANDLE		     hMampF_CLI_Xsens;
HANDLE		     hMampF_CLI_NovAGPS;
HANDLE		     hMampF_CLI_NovAVEL;

Lidar_smdat* CLI_smdat_Lidar;
Xsens_smdat* CLI_smdat_Xsens;
NovATel_smdat* CLI_smdat_NovAGPS;
NovATel_smdat* CLI_smdat_NovAVEL;

//===============================
// Buffer
//===============================
double buf_Raw_Roll[BUF_SIZE] = { 0, };
double buf_Raw_Pitch[BUF_SIZE] = { 0, };
double buf_Raw_Yaw[BUF_SIZE] = { 0, };
double buf_Raw_AccX[BUF_SIZE] = { 0, };
double buf_Raw_AccY[BUF_SIZE] = { 0, };
double buf_Raw_AccZ[BUF_SIZE] = { 0, };

double buf_Raw_Lat[BUF_SIZE] = { 0, };
double buf_Raw_Lon[BUF_SIZE] = { 0, };
double buf_direction[BUF_SIZE] = { 0, };
double buf_velocity[BUF_SIZE] = { 0, };
//double buf_velRead   [BUF_SIZE] = { 0, };

int    buf_FixQual[BUF_SIZE] = { 0, };

#endif 