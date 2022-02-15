#include "include.h"
#include "function.h"

double test = 1.0;
double task1, task2;
extern double  yaw;
extern int CHOOSE_PARKING_IDX;
int mission;

double t1 = 0.0;
double t2 = 0.0;
double t3 = 0.0;

void main()
{
	//mission = 1;

	CHOOSE_PARKING_IDX = 13 ;  //몇번째 칸인지 적어주세요  8 ~ 13, 
	MOD_select     (Xsens_ON  , Novatel_ON,  Lidar_ON, CamObj_ON, CamLin_OFF,
		            Global_ON,  Local_ON, ERP_ON, Parking_OFF,Cluster_OFF, Delivery_ON );	
	Mission_select (1, 1, 1, 1);
	Sensor_Open    ();
	Initialization ();
	wait_UDP       (RecvArray_trafficSign, RecvArray_Line);
	printf("===========START!!===========\n");
	vel_cmd = MAX_SPEED;
	brk_cmd = 1;
	InitTImeCheck ();
	
	do
	{
		vel_cmd = MAX_SPEED;
		brk_cmd = 1;

		t1 = GetWindowTime();
		
		ImportData      ();
		Mission_Flag    ();    // Camera와 통신 체크 시 주석처리
		Parking         ();
		
		Global_Guidance ();
		Local_Guidance  ();

		Mission_Command();
		CMD_Guidance2erp(vel_cmd, brk_cmd);

		Idle_Time       ();
		if (_kbhit()) 
		{ 
			printf("tddd\n"); 
			key = _getch(); 
			if (key == 'f')
			break; 
		}

	}while (simu_t < FINAL_TIME);

	CloseEVERYTING();

	SaveData();
}
