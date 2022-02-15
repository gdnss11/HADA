#pragma once
#ifndef _COMMON_VARIABLE
#define _COMMON_VARIABLE


//===============================
// ERP 42 PROTOCOL LENGTH
#define nWRITE (uint8_t)(14)
#define nREAD  (uint8_t)(18)

// ERP 42 PROTOCOL DEFAULT SETTING
#define S (uint8_t)(0x53)
#define T (uint8_t)(0x54)
#define X (uint8_t)(0x58)

#define AorM_Manual   (uint8_t)(0x00)
#define AorM_AutoMode (uint8_t)(0x01)

#define ESTOP_ON  (uint8_t)(0x01)
#define ESTOP_OFF (uint8_t)(0x00)

#define GEAR_FOWARD   (uint8_t)(0x00)
#define GEAR_NEUTRAL  (uint8_t)(0x01)
#define GEAR_BACKWORD (uint8_t)(0x02)

#define ETX0 (uint8_t)(0x0D)
#define ETX1 (uint8_t)(0x0A)

#define SPEED0 (uint8_t)(0x00) // range      0 ~ 200
#define SPEED1 (uint8_t)(0x00) // range      0 ~ 200
#define STEER0 (uint8_t)(0x00) // range  -2000 ~ 2000
#define STEER1 (uint8_t)(0x00) // range  -2000 ~ 2000
#define BRAKE  (uint8_t)(0x01) // range      1 ~ 200
#define ALIVE  (uint8_t)(0x00) // range       0 ~ 255


//===============================
// Time check define

#define      FINAL_TIME             2000.0    //[sec]
#define      RTK_INITIAL_TIME        5.0    //[sec]
#define      XSENS_INITIAL_TIME      5.0    //[sec]
#define      POS_INITIAL_TIME       10.0    //[sec]
#define      SAMPLING_TIME          0.05    //[sec]
#define      BUF_SIZE				(int)(FINAL_TIME/SAMPLING_TIME)

//===============================
// UNIT
#define      PI                 3.1415926535
#define      DEG2RAD            PI/180.0
#define      RAD2DEG            180.0/PI

//#define      LAT2METER (double) 110950.59672489
//#define      LON2METER (double) 90048.170449268  // HANDONG

#define      LAT2METER (double) 110975.575908909
#define      LON2METER (double) 88743.5932955675  // KCITY


//==============================
// Global Guidance

#define      IDX_STANDARD           130
#define      BODY_L                 1.0
#define      K_GLOB                 1.5//5// // 
#define      MAX_STEER             28.0
#define      MIN_DIST               3.0

#define      MAX_SPEED             150

#define      MID_SPEED              80.0
#define      SLOW_SPEED             70.0
#define      STOP_SPEED              0.0
//#define      PALGACK_YAW            19.0//-161.0
#define      NEW2CHA                 19.0   // Newton(팔각정 기준)에서 채플가는 길
#define      NEW2GAL                -161.0  // Newton(팔각정 기준)에서 갈상가는 길
#define      CHAPEL_AP              -71     // 채플 앞에서 obstacle확인할 때 쓰는 초기자세값

#define     HAN_DELDIST          1.5       //[m]
#define     KCITY_DELDIST        2.0       //[m]
#define     HAN_POSE              19
#define     KCITY_POSE            -2


#define      HWASUNG_YAW            28.0

//==============================
// Local Guidance
#define      RangeMAX             3000           //500 per 1[m]
#define      K_LOCAL              0.16           // {scalefactor(500) * meter} / {scalefactor(500)*max_range}
#define      MAX_NEIGHBOR_NUM   520000
#define      MAX_LIDAR_NUM	       761	         //500360
#define      CLUSTER_NUM           721
#define      MAX_ANGLE_POSITIVE     28 * DEG2RAD
#define      MAX_ANGLE_NEGATIVE    -28 * DEG2RAD


//===============================
// Mission Algorithm
enum DeliveryIDX { DelA = 0, DelB , StartIDX = 0, FinalIDX = 1};
enum ParkingMODE { PARKING_INIT = 0, TRAJECTORY, KANAYAMA, PARKING_STOP, BACKOUT, STOP_TO_GO };
enum VehicleGEAR { Drive = 0, Neutral, Reverse };

#define NPARKINIT     (int)(20)
#define K_PARKX       (double)(6.0)
#define K_PARKY       (double)(1.0)
#define K_PARKTH      (double)(0.6)
#define PARK_VEL_REF  (double)(60.0)     //[km/h]
#define PARK_W_REF    (double)(0.0)
#define PARKTIME      (double)(10.0)
#define BACK_MIN_DIST (double)(6.0)

#define R_PARK                (double)(3.0)// 원의 반지름
#define N_points_line_init    (unsigned int)(10)	   // 첨 직선에 위치한 점의 수
#define N_points_one          (unsigned int)(10)	   // 원의 호에 위치한 점의 수
#define N_points_line_goal    (unsigned int)(10)	   // 끝 직선에 위치한 점의 수
#define NParkingWaypoint (unsigned int)(N_points_line_init + N_points_one + N_points_line_goal) // 총 점


//===============================
// Delivery Design Parmeters
#define THRESH_DIST (double)(0.8) // [m] Distance which car have to go while delivery mission


#define mode_print_del (int)(0) // print delivery information (1 == ON) (0 == OFF) 



//===============================
// MOD commend
#define      SteerMODE      1
#define      SpeedMODE      1
#define      GearMODE       0

enum ERP_Modselect { ERP_OFF, ERP_ON };
enum Guidance_Mod { Global_OFF = 0, Local_OFF = 0, Global_ON = 1, Local_ON = 1 };
enum Sensor_Mod {
	Xsens_OFF = 0, Novatel_OFF = 0, Lidar_OFF = 0, Camera_OFF = 0, CamObj_OFF = 0, CamLin_OFF = 0, CamRu_OFF = 0,
	Xsens_ON = 1, Novatel_ON = 1, Lidar_ON = 1, Camera_ON = 1, CamObj_ON = 1, CamLin_ON = 1, CamRu_ON = 1
};
enum Traffic_flag { FLAG_STOP = 0, FLAG_GO, FLAG_LEFT, FLAG_GOLEFT, FLAG_ERROR};
enum Delivery_flag {A1 = 6, A2, A3, B1, B2, B3, NO_FLAG = 18};
enum Mission_Mod { Parking_OFF = 0, Parking_ON = 1, Cluster_ON = 1, Cluster_OFF = 0,
				   Delivery_OFF = 0, Delivery_ON = 1, Obstacle_OFF = 0, Obstacle_ON = 1};
//extern void PlatformControl(void);
enum ERP_Protocol {
	eS = 0,
	eT, eX, eAUTO, eESTOP, eGEAR, eSPEED0, eSPEED1, eSTEER0, eSTEER1, eBRAKE, eALIVE, eETX0, eETX1
};

enum FLAG { COPLIETE = -1, STOP = 0, INIT = 1, START = 2 };


//===============================
// Local Guidance
//#define MAX_LASER_POINT	     (unsigned int)(       761)
#define MAX_LASER_RECEIVE	 (unsigned int)(   16*1024)
#define MAX_LIDAR_NUM         761
//===============================
// Clustering
#define minPoint       (int)    10
#define boundary       (double) 0.1

//===============================
// Macro Function
#define SQUA(a) a*a
#define CUBE(a) a*a*a
#define QUAD(a) a*a*a*a

#define FMOD(STAND,SAMPLE_T,TYPE,VARI)		\ if(fmod(STAN,SAMPLE_T)==0){printf("%TYPE\n", VARI);};

#define COL_NUM(MAT)						 sizeof(MAT[0])/sizeof(double)
#define ROW_NUM(MAT)						 sizeof(MAT)/sizeof(MAT[0])
#define MAT_PLUS(MAT_f,MAT_r,output)		 for(int i=0;i<ROW_NUM(MAT_f);i++){for(int k=0;k<COL_NUM(MAT_f);k++){output[i][k]=(MAT_f[i][k] + MAT_r[i][k]);}}
#define MAT_MINUS(MAT_f,MAT_r,output)	     for(int i=0;i<ROW_NUM(MAT_f);i++){for(int k=0;k<COL_NUM(MAT_f);k++){output[i][k]=(MAT_f[i][k] - MAT_r[i][k]);}}
#define MAT_MULTI(MAT_f,MAT_r,output)		 for(int i=0;i<ROW_NUM(MAT_f);i++){for(int j=0;j<COL_NUM(MAT_r);j++){output[i][j]=0;for(int k=0;k<COL_NUM(MAT_f);k++)output[i][j]+=(MAT_f[i][k]   *MAT_r[k][j]);}}
#define MAT_MULTI_TS(MAT_f,MAT_r,TS,output)  for(int i=0;i<ROW_NUM(MAT_f);i++){for(int j=0;j<COL_NUM(MAT_r);j++){output[i][j]=0;for(int k=0;k<COL_NUM(MAT_f);k++)output[i][j]+=(TS*MAT_f[i][k]*MAT_r[k][j]);}}
#define EXIT_LOOP                            if(exitOuterLoop == true)
//#define WAY_SECTOR(START, END, SPEED, STATE, CMD, )        if(idx_max[START] <= idx_way < idx_max[END]){speedcmd = SPEED}

#define LOOP(Count,End)                      for( Count=0; Count<End; Count++ )
#define VEC_PRT_OFF(X,UNT )				     {LOOP(j,GET_DIM(X[i])) fprintf( offFile, "%14.5f\t", X[i][j]*UNT  );}
#define STR_PRT_OFF(X     )                  {fprintf( offFile, "%s",   X                );}
#define GET_DIM(X)							 (sizeof(X) / sizeof(X[0]))
#define VEC_WRT(BUF,ARAY,COUNTO)		     {LOOP(i, GET_DIM(ARAY)) BUF[COUNTO][i] = ARAY[i];}
#define KEY_HIT(KEY)    		             if (_kbhit()) {printf("tddd\n");key = _getch();if (key == 'f');break;}


#endif // !COMMON
