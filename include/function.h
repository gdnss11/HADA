#pragma once
int count = 0;
int key;
int xsensMOD , novatelMOD, lidarMOD  , erpMOD,
    globalMOD, localMOD  , parkingMOD,
    camobMOD , camlaneMOD, clusterMOD, deliveryMOD, obstacleMOD, trafficsignMOD= 0;

#include "ExternVariable.h"
void Mission_select(int m_park, int m_obs, int m_sign, int m_del);
void MOD_select(int _xsens, int _novatel, int _lidar, int _camObject, int _camLine, int _global, int _local, int _erp, int _parking, int _cluster, int _deliver);

//============================//
//            Filter          //
//============================//
void variable();
void NavigationFilter();
void SaveData_Filter();

//============================//
//       Global Guidance      //
//============================//
void ReadWaypoint();
void SaveMissionPoint();

void FieldBuffer();
void Xsens_Initialization();
void NovAtel_Initialization();
void Select_Xsens_yaw();

void Pursuit_Guidance();
void WayPoint_Change();
void SaveData_Guidance();


//============================//
//        ERP42 command       //
//============================//
void Open_ERPconnect ();
void Read_ERP42      ();
void CMD_Guidance2erp(int _spdcmd, int _brkcmd);
void CMD_delta_f     ();
void SaveData_ERP    ();
extern void RS232_CloseComport(int comport_number);


//============================//
//        Local Guidance      //
//============================//
void ParameterSetting();
void FieldBuffer();
void GenerateOFF();
void GenerateSFF();
void GenerateIFF();
void Lidar_Clustering();

void SaveData_Lidar();


//============================//
//      Mission Algorithm     //
//============================//
void SelectParkingPosition();
void ParkingMission();
void SaveData_Parking();


//============================//
//          Time Check        //
//============================//
double GetWindowTime();
void InitTImeCheck();
void Idle_Time();

//============================//
//          Data Read         //
//============================//
void OpenSharedMemory();
void Import_Lidar();
void Import_Xsens();
//void Import_BestPos();
void Import_GPGGA();
void Import_BestVel();
void Import_Novatel()
{
	//Import_BestPos();
	Import_GPGGA  ();
	Import_BestVel();
}
void SaveData_Raw ();
void SaveParameter();


void Import_Data()
{
	Import_Xsens();
	Import_Novatel();
	Import_Lidar();
}
void Close_SharedMemory();


//============================//
//        UDP Function        //
//============================//
void Open_UDP_Socket   ();
void wait_UDP          (int _recvData_Sign[], double _recvData_Line[]);
void Recvdata_UDP_Sign (int* _recvData);
void Recvdata_UDP_Lane (double* _recvData, double _sendData);
void Close_UDP         ();
//void SendData          ();
void SaveData_UDP      ();

//=============================//
//			Mission			   //
//=============================//
void Mission_Flag();
void Mission_Command();


void Sensor_Open()
{
	Open_ERPconnect  ();
	OpenSharedMemory ();
	Open_UDP_Socket  ();
}

void Initialization()
{
	ReadWaypoint    ();
	SaveMissionPoint();
	FieldBuffer     ();

	if (novatelMOD)
	{
		NovAtel_Initialization();
		variable();
	}

	SelectParkingPosition();

	if (xsensMOD)
		Xsens_Initialization();
}

void SaveData()
{
	SaveData_Raw     ();
	SaveData_Guidance();
	SaveData_Filter  ();
	SaveParameter    ();
	SaveData_Parking();

	if (lidarMOD)
		SaveData_Lidar();
	if (camlaneMOD || camobMOD == 1)
		SaveData_UDP();
	if (erpMOD)
		SaveData_ERP();
}

void ImportData()
{
	if (xsensMOD)
		Import_Xsens();
	if (novatelMOD)
		Import_Novatel();
	if (lidarMOD)
		Import_Lidar();
	if (camobMOD)
		Recvdata_UDP_Sign(RecvArray_trafficSign);
	if (camlaneMOD)
		Recvdata_UDP_Lane(RecvArray_Line, velocity); //Camera to Guidance Delta_f & flag
}

void Global_Guidance()
{
	if (globalMOD)
	{
		
		NavigationFilter ();   //
		Pursuit_Guidance ();
		WayPoint_Change();
		
	}
	if (camobMOD)
	{
		if (fixqual != 4 && fixqual != 5 && RecvArray_Line[1] == 1.0) // for real situation
		//if (RecvArray_Line[1] == 1.0 && 42 <= idx_way && idx_way <= 69) // for 0922 morning experiment
		{
			delta_f = RecvArray_Line[0];
		}
	}
}

void CloseEVERYTING()
{
	if (erpMOD)
		RS232_CloseComport(cport_nr);
	Close_SharedMemory();
	Close_UDP         ();
}

void Local_Guidance()
{
	if (localMOD)
	{
		//printf("/\n");
		ParameterSetting();
		//printf("%d\n", localMOD);
		//printf("dkdk\n");
		GenerateOFF();
		GenerateSFF();
		GenerateIFF();
	}

	//if (clusterMOD)
	//{
	//	GenerateOFF();
	//	//Lidar_Clustering();
	//}
}

void Parking()
{
	if (idx_way < idx_park)
	{
		vel_cmd = 100;
		//printf("parking Vel : %d\n", vel_cmd);
	}

	if (parkingMOD)
	{

		NavigationFilter();
		ParkingMission ();

		printf("STOP_IDX: %d\n", stop_idx);
		if (stop_idx > 30) {
			parkingMOD = Parking_OFF;
			globalMOD = Global_ON;
			localMOD = Local_ON;

		}
		if (stop_idx == 31)
		{
			Min_dist = 10000;
			for (int i = 0; i < 40; i++)
			{
				//cnt++;
				//printf("cn: %d\n", cnt);
				Way_dist = sqrt(pow(GPS_waypoint_Lat[i] * LAT2METER - lat_filtered * LAT2METER, 2) + pow(GPS_waypoint_Lon[i] * LON2METER - lon_filtered * LON2METER, 2));
				if (Min_dist > Way_dist)
				{
					Min_dist = Way_dist;
					Start_point = i;
				}
			}
			idx_park = Start_point;
			//idx_max[0] = idx_max[0] - Start_point;
			printf("Start idx:%d\n", Start_point);

			Start_point  = Start_point + 2;

			idx_way = Start_point;
			//idx_way = 39;
			vel_cmd = MAX_SPEED;
			brk_cmd = 1;
			printf("idx_way: %d============================\n", idx_way);
			GPS_waypoint_Lat_now = GPS_waypoint_Lat[Start_point];
			GPS_waypoint_Lon_now = GPS_waypoint_Lon[Start_point];
		}

	}
}