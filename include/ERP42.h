#ifndef _HEADER_ERP42
#define _HEADER_ERP42

#include "include.h"

//=======================================
// ERP 42 cmd
char         mode[] = { '8','N','1',0 };
char         writebuffer[nWRITE] = { S,T,X,AorM_AutoMode,ESTOP_OFF,GEAR_FOWARD,SPEED0,SPEED1,STEER0,STEER1,BRAKE,ALIVE,ETX0,ETX1 };
char         readbuffer[nREAD] = { 0.0, };
	        
int          cport_nr = 4, bdrate = 115200;  /*com3 => cport_nr = 2*/
int          steercmd = 0;
int          speedcmd = 0;
int          write_len = 0;
int          read_len = 0;
double       vel_read = 0.0;
double       steer_read = 0.0;

unsigned int idx_alive = 0x00;
unsigned int idx_ndata = 0;

double buf_velRead  [BUF_SIZE] = { 0, };
double buf_steerRead[BUF_SIZE] = { 0, };

//=======================================
// Guidance related

#endif