#ifndef UDP_COMMUNICATE
#define UDP_COMMUNICATE

#include "UDP.h"


void ErrorHandling(char* message);

// Start & Set WinSock
void Open_UDP_Socket()
{
	if (camlaneMOD)
	{
		if (WSAStartup(MAKEWORD(2, 2), &wsaData1) != 0)
			ErrorHandling("WSAStartup() eror!");

		servSock_Lane = socket(PF_INET, SOCK_DGRAM, 0);
		if (servSock_Lane == INVALID_SOCKET)
			ErrorHandling("UDP socket creation error");

		memset(&servAdr_Line, 0, sizeof(servAdr_Line));
		servAdr_Line.sin_family = AF_INET;
		servAdr_Line.sin_addr.s_addr = htonl(INADDR_ANY);
		servAdr_Line.sin_port = htons(Port_Line);

		if (bind(servSock_Lane, (SOCKADDR*)&servAdr_Line, sizeof(servAdr_Line)) == SOCKET_ERROR)
			ErrorHandling("BIND()_Line error");

		printf("UDP setting Complete 1!\n");
	}

	if (camobMOD)
	{
		if (WSAStartup(MAKEWORD(2, 2), &wsaData2) != 0)
			ErrorHandling("WSAStartup() eror!");

		servSock_Sign = socket(PF_INET, SOCK_DGRAM, 0);
		if (servSock_Sign == INVALID_SOCKET)
			ErrorHandling("UDP socket creation error");

		memset(&servAdr_Sign, 0, sizeof(servAdr_Sign));
		servAdr_Sign.sin_family = AF_INET;
		servAdr_Sign.sin_addr.s_addr = htonl(INADDR_ANY);
		servAdr_Sign.sin_port = htons(Port_Sign);

		if (bind(servSock_Sign, (SOCKADDR*)&servAdr_Sign, sizeof(servAdr_Sign)) == SOCKET_ERROR)
			ErrorHandling("BIND()_Sign error");

		printf("UDP setting Complete 2!\n");
	}
}


//void SendData()
//{
//	// write to RS232
//	// printf("COMMAND BREAK: %d\n", brk_cmd);
//
//	CMD_Guidance2erp(vel_cmd, brk_cmd);
//}

void wait_UDP(int _recvData_Sign[], double _recvData_Line[])
{
	if (camlaneMOD)
	{
		printf("===========Waiting UDP_Lane...===========\n");
		//do
		//{
		printf("I'm waiting...\n");
		clntAdrSz = sizeof(clntAdr_Lane);
		nread_UDP_NAVI = recvfrom(servSock_Lane, _recvData_Line, sizeof(_recvData_Line), 0,
			(SOCKADDR*)&clntAdr_Lane, &clntAdrSz);

		//} while (_recvData_Line[0] != 1 && _recvData_Line[0] != 0);

		printf("UDP_Lane connected!\n");
	}

	if (camobMOD)
	{
		printf("===========Waiting UDP_Sign...===========\n");
		do
		{
			clntAdrSz = sizeof(clntAdr_Sign);
			nread_UDP_NAVI = recvfrom(servSock_Sign, _recvData_Sign, sizeof(_recvData_Sign), 0,
				(SOCKADDR*)&clntAdr_Sign, &clntAdrSz);
			printf("getting udp object\n");
		} while (_recvData_Sign[0] != 18 && _recvData_Sign[0] != 0);

		printf("UDP_Sign connected!\n");
	}

	//if (camobMOD)
	//{
	//	sendto(servSock_Sign, &send_ob, sizeof(send_ob), 0, (SOCKADDR*)&clntAdr_Sign, sizeof(clntAdr_Sign));
	//	printf("Send UDP Sign sucessful!\n");
	//}

	// Stanely 를 사용하지 않기 때문에 이제는 필요가 없다. 
	//if (camlaneMOD)
	//{
	//	sendto(servSock_Lane, &velocity, sizeof(velocity), 0, (SOCKADDR*)&clntAdr_Lane, sizeof(clntAdr_Lane));
	//	printf("Send UDP Lane sucessful!\n");
	//}
}

// Preliminary - Rubber cone
void Recvdata_UDP_Sign(int* _recvData)
{
	if (fmod(simul_cnt, 2) == 0) // system 20hz / sign 10hz
	{
		clntAdrSz = sizeof(clntAdr_Sign);
		nread_UDP_NAVI = recvfrom(servSock_Sign, _recvData, 32, 0,
			(SOCKADDR*)&clntAdr_Sign, &clntAdrSz);

		//printf("Traffic Sign flag : %d\n", _recvData[0]);
	}
	buf_signflag[0][count] = _recvData[0]; // traffic sign // 안잡으면 18
	buf_signflag[1][count] = _recvData[1]; // 장애인 주차  // 안잡으면 18   // 잡으면 1 (true)
	buf_signflag[2][count] = _recvData[2]; // 배달 표지판 // 안잡으면 18 A1 // 잡으면 6 (true)
	buf_signflag[3][count] = _recvData[3]; // 배달 표지판 // 안잡으면 18 A2 // 잡으면 7 (true)
	buf_signflag[4][count] = _recvData[4]; // 배달 표지판 // 안잡으면 18 A3 // 잡으면 8 (true)
	buf_signflag[5][count] = _recvData[5]; // 배달 표지판 // 안잡으면 18 B1 // 잡으면 9 (true)
	buf_signflag[6][count] = _recvData[6]; // 배달 표지판 // 안잡으면 18 B2 // 잡으면 10 (true)
	buf_signflag[7][count] = _recvData[7]; // 배달 표지판 // 안잡으면 18 B3 // 잡으면 11 (true)

	Delivery_flagA1 = _recvData[2];
	Delivery_flagA2 = _recvData[3];
	Delivery_flagA3 = _recvData[4];
	Delivery_flagB1 = _recvData[5];
	Delivery_flagB2 = _recvData[6];
	Delivery_flagB3 = _recvData[7];

	//printf("traffic sign : %d\t 장애인 주차 : %d\t, 배달 표지판 A : %d, 배달 표지판 B : %d\n", _recvData[0], _recvData[1], _recvData[2], _recvData[3]);
	printf(" A1 : %d, A2 : %d, A3 : %d,  B1 : %d, B2 : %d, B3 : %d\n", _recvData[2], _recvData[3], _recvData[4], _recvData[5], _recvData[6], _recvData[7]);

}

// Camera - Stanley
void Recvdata_UDP_Lane(double* _recvData, double _sendData)
{

	if (fmod(simul_cnt, 4) == 0) // system 20hz / sign 10hz
	{
		if (camlaneMOD)
		{
			clntAdrSz = sizeof(clntAdr_Sign);
			nread_UDP_NAVI = recvfrom(servSock_Lane, _recvData, 16, 0,
				(SOCKADDR*)&clntAdr_Sign, &clntAdrSz);
			//printf("Stanley deltaf : %f \n", _recvData[0]);
			//printf("lane flag : %f \n", _recvData[1]);

			buf_laneflag[0][count] = _recvData[0];
			buf_laneflag[1][count] = _recvData[1];
			delta_f = _recvData[0];
		}
	}
}


// Close & Error UDP
void ErrorHandling(char* message)
{
	fputs(message, stderr);
	fputc('\n', stderr);
	exit(1);
}

void Close_UDP()
{
	if (camlaneMOD)
		closesocket(servSock_Lane);
	if (camobMOD)
		closesocket(servSock_Sign);
	WSACleanup();
}

void SaveData_UDP()
{
	char Filename[20];// = "_UDPdata.csv";
	strcpy(Filename, "_UDPdata.csv");

	char Filepath[250] = "C:\\Users\\HADA\\Desktop\\최신구현코드\\kcity0929_완전최신\\kcity0929_완전최신\\REVISION_0911\\Data\\";
	char time_c[256];
	time_t rawTime;
	struct tm pTimeInfo;

	setlocale(LC_ALL, "Korean");
	time(&rawTime);
	localtime_s(&pTimeInfo, &rawTime);
	strftime(time_c, sizeof(time_c), "%m%d_%H%M", &pTimeInfo);

	strcat(time_c, Filename);
	strcat(Filepath, time_c);
	dataFile = fopen(Filepath, "wt");

	if (dataFile == NULL) {
		printf("안됨..\n");
		return 1;
	}

	for (int r = 0; r < count; r++) {
		fprintf(dataFile, "%d, %.7f, %.7f, %.7f\n",
			(int)buf_signflag[r], (double)buf_laneflag[0][r], (double)buf_laneflag[1][r], (double)buf_vel4flag[r]);
	}

	printf("Save UDP flag Successful.\n");

	fclose(dataFile);
}


#endif // !UDP_COMMUNICATE
