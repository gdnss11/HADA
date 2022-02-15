#pragma once
#ifndef MEMWRITE
#define MEMWRITE

extern int buf_Index2[];
#include "memwrite.h"

void OpenSharedMemory()
{
	if (xsensMOD)
		OpenClientXsens();

	if (novatelMOD)
	{
		OpenClientNovA_GPS();
		OpenClientNovA_VEL();
	}

	if (lidarMOD)
		OpenClientLidar();
}

void OpenClientXsens(void)
{
	TCHAR CLIE_XSENS[] = TEXT("Xsens_smdat_ReadData");

	hMampF_CLI_Xsens = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, CLIE_XSENS);
	if (hMampF_CLI_Xsens == NULL)
	{
		printf("Xsens   : Could not open file mapping object (%d).\n", GetLastError());
		return 1;
	}

	CLI_smdat_Xsens = (Xsens_smdat*)MapViewOfFile(hMampF_CLI_Xsens, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(Xsens_smdat));

	if (CLI_smdat_Xsens == NULL)
	{
		printf("Xsens   : Could not map view of file (%d).\n", GetLastError());
		CloseHandle(hMampF_CLI_Xsens);
		return 1;
	}
}

void OpenClientNovA_GPS(void)
{
	TCHAR CLIE_NOVAGPS[] = TEXT("NovATelGPS_smdat_ReadData");

	hMampF_CLI_NovAGPS = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, CLIE_NOVAGPS);
	if (hMampF_CLI_NovAGPS == NULL)
	{
		printf("NovA GPS: Could not open file mapping object (%d).\n", GetLastError());
		return 1;
	}

	CLI_smdat_NovAGPS = (NovATel_smdat*)MapViewOfFile(hMampF_CLI_NovAGPS, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(NovATel_smdat));

	if (CLI_smdat_NovAGPS == NULL)
	{
		printf("NovA GPS: Could not map view of file (%d).\n", GetLastError());
		CloseHandle(hMampF_CLI_NovAGPS);
		return 1;
	}
}

void OpenClientNovA_VEL(void)
{
	TCHAR CLIE_NOVAVEL[] = TEXT("NovATelVEL_smdat_ReadData");

	hMampF_CLI_NovAVEL = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, CLIE_NOVAVEL);
	if (hMampF_CLI_NovAVEL == NULL)
	{
		printf("NovA Vel: Could not open file mapping object (%d).\n", GetLastError());
		return 1;
	}

	CLI_smdat_NovAVEL = (NovATel_smdat*)MapViewOfFile(hMampF_CLI_NovAVEL, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(NovATel_smdat));

	if (CLI_smdat_NovAVEL == NULL)
	{
		printf("NovA Vel: Could not map view of file (%d).\n", GetLastError());
		CloseHandle(hMampF_CLI_NovAVEL);
		return 1;
	}
}

void OpenClientLidar(void)
{
	TCHAR CLIE_LIDAR[] = TEXT("Lidar_smdat_ReadData");

	hMampF_CLI_Lidar = OpenFileMapping(
		FILE_MAP_READ,            // read/write access
		FALSE,                    // do not inherit the name
		CLIE_LIDAR);               // name of mapping object RP_SMDAT

	if (hMampF_CLI_Lidar == NULL)
	{
		_tprintf(TEXT("Lidar   : Could not open file mapping object (%d).\n"),
			GetLastError());
		exit(1);
		return 1;
	}

	CLI_smdat_Lidar = (Lidar_smdat*)MapViewOfFile(hMampF_CLI_Lidar, FILE_MAP_READ, 0, 0, sizeof(Lidar_smdat));

	if (CLI_smdat_Lidar == NULL)
	{
		_tprintf(TEXT("Lidar   : Could not map view of file (%d).\n"),
			GetLastError());
		exit(1);
		CloseHandle(hMampF_CLI_Lidar);
		return 1;
	}

	//printf("sm_connect\n");

}

void Close_SharedMemory()
{
	if (xsensMOD)
	{
		UnmapViewOfFile(CLI_smdat_Xsens);
		CloseHandle(hMampF_CLI_Xsens);
	}
	if (novatelMOD)
	{
		UnmapViewOfFile(CLI_smdat_NovAVEL);
		CloseHandle(hMampF_CLI_NovAVEL);
	}
	if (novatelMOD)
	{
		UnmapViewOfFile(CLI_smdat_NovAGPS);
		CloseHandle(hMampF_CLI_NovAGPS);
	}
	if (lidarMOD)
	{
		UnmapViewOfFile(CLI_smdat_Lidar);
		CloseHandle(hMampF_CLI_Lidar);
	}
}


void Import_Xsens()
{
	//index_imu = CLI_smdat_Xsens->SIM_count;
	roll = CLI_smdat_Xsens->euler_x;
	pitch = CLI_smdat_Xsens->euler_y;
	yaw = CLI_smdat_Xsens->euler_z;
	yaw   = yaw + yaw_OFFSET;
	//printf("%f\n", yaw);

	if (yaw > 180.0)
		yaw = yaw - 360.0;
	else if (yaw < -180.0)
		yaw = yaw + 360.0;

	accX = CLI_smdat_Xsens->acc_x;
	accY = CLI_smdat_Xsens->acc_y;
	accZ = CLI_smdat_Xsens->acc_z;

	buf_Raw_Roll[count] = roll;
	buf_Raw_Pitch[count] = pitch;
	buf_Raw_Yaw[count] = yaw;
	buf_Raw_AccX[count] = accX;
	buf_Raw_AccY[count] = accY;
	buf_Raw_AccZ[count] = accZ;
}

void Import_GPGGA()
{
	index_gps = CLI_smdat_NovAGPS->SIM_count;
	latitude = CLI_smdat_NovAGPS->LatDegree + CLI_smdat_NovAGPS->LatMinute / 60.0;
	longitude = CLI_smdat_NovAGPS->LongDegree + CLI_smdat_NovAGPS->LongMinute / 60.0;
	fixqual = CLI_smdat_NovAGPS->FixQual;

	//printf("%d\t%f\t%d\t%f\n", CLI_smdat_NovAGPS->LatDegree, CLI_smdat_NovAGPS->LatMinute, CLI_smdat_NovAGPS->LongDegree, CLI_smdat_NovAGPS->LongMinute);
	//printf("%f\t%f\n", velocity, direction);

	buf_Raw_Lat[count] = latitude;
	buf_Raw_Lon[count] = longitude;
	buf_FixQual[count] = fixqual;
	//printf("LAT: %f\tLON: %f\n", latitude, longitude);
}

//void Import_BestPos()
//{
//	latitude  = CLI_smdat_NovAGPS->BestLat;
//	longitude = CLI_smdat_NovAGPS->BestLon;
//	fixqual = CLI_smdat_NovAGPS->FixQual;
//
//	buf_Raw_Lat[count] = latitude;
//	buf_Raw_Lon[count] = longitude;
//
//	buf_FixQual[count] = fixqual;
//	printf("%f\t%f\t%d\n", CLI_smdat_NovAGPS->BestLat, longitude, fixqual);
//}

void Import_BestVel()
{
	direction = CLI_smdat_NovAVEL->Direction;
	velocity = CLI_smdat_NovAVEL->Velocity;

	if (direction > 180.0)
		direction = direction - 360.0;
	else if (direction < -180.0)
		direction = direction + 360.0;


	buf_direction[count] = direction;
	buf_velocity[count] = velocity;
}


void Import_Lidar()
{
	for (int m = 0; m < 761; m++)
	{
		Lidar_dis[m] = CLI_smdat_Lidar->dist[m];
	}
}



void SaveData_Raw()
{
	char Filename[15];
	strcpy(Filename, "_RawData.csv");

	char Filepath[250] = "C:\\Users\\HADA\\Desktop\\최신구현코드\\GUI_LAST\\kcity1001_shinhohochingut\\kcity0929_new\\REVISION_0911\\Data\\";
	char time_c[256];
	time_t rawTime;
	struct tm pTimeInfo;

	setlocale(LC_ALL, "Korean");
	time(&rawTime);
	localtime_s(&pTimeInfo, &rawTime);
	strftime(time_c, sizeof(time_c), "%m%d_%H%M", &pTimeInfo);

	strcat(time_c, Filename);
	strcat(Filepath,time_c);
	//printf("SaveData_Raw의 Filepath : %s\n", Filepath);
	dataFile = fopen(Filepath, "wt");

	if (dataFile == NULL) {
		printf("안됨..\n");
		return 1;
	}

	for (int r = 0; r < count; r++) {
		fprintf(dataFile, "%d, %.7f, %.7f, %c, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f\n",
			(int)buf_Index2      [r], (double)buf_Raw_Lat  [r], (double)buf_Raw_Lon [r], (int)buf_FixQual[r],
			(double)buf_Raw_Roll [r], (double)buf_Raw_Pitch[r], (double)buf_Raw_Yaw [r],
			(double)buf_Raw_AccX [r], (double)buf_Raw_AccY [r], (double)buf_Raw_AccZ[r],
			(double)buf_direction[r], (double)buf_velocity [r]//f, (double)buf_velRead[r]
		);

	}
	printf("Raw data Successful\n");
	fclose(dataFile);
}
#endif // !MEMWRITE
