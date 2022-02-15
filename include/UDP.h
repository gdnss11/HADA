#ifndef UDP_COMMUN
#define UDP_COMMUN

#include "Include.h"

#define LEFT_CONE   1
#define RIGHT_CONE  2
extern double delta_f;
extern int   fixqual;

typedef unsigned short PORT;

// socket variable
WSADATA wsaData1;
SOCKET servSock_Lane;
WSADATA wsaData2;
SOCKET servSock_Sign;

//double UDP_NAVI_recvbuff[NUDP_NOVATEL] = { 0.0 };
int    RecvArray_rubber[1] = { 0 };      // rubber cone fail 유무 0 -성공 / 1-왼쪽 / 2-오른쪽 못잡음 
int    RecvArray_trafficSign[8] = { 0, };      // [0] : traffic sign // [1] : 장애인 주차 // [2] : 배달 표지판 A //  [3] : 배달 표지판 B // 안잡으면 18
double RecvArray_Line[2] = { 0.0, };   // stanley deltaf / line yes or no 

int    buf_signflag[8][BUF_SIZE] = { 0.0 };
double buf_vel4flag[BUF_SIZE] = { 0.0 };
double buf_laneflag[2][BUF_SIZE] = { 0.0 };

double Sendvel = 0.0; // 도플러 속도
int send_ob = 18;

int nread_UDP_NAVI = 0; // n_ : length 
int clntAdrSz = 0; // _sz : sizeof

SOCKADDR_IN servAdr_Line, clntAdr_Lane;
SOCKADDR_IN servAdr_Sign, clntAdr_Sign;

PORT Port_Line = 1101;
PORT Port_Sign = 2101;


#endif // !UDP_COMMUN
