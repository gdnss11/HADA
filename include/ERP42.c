#ifndef ERP_COMMAND_C
#define ERP_COMMAND_C

#include "ERP42.h"
extern int RS232_OpenComport(int comport_number, int baudrate, const char* mode, int flowctrl);
extern int RS232_SendBuf    (int comport_number, unsigned char* buf, int size);
extern int RS232_PollComport(int comport_number, unsigned char* buf, int size);

void Open_ERPconnect()
{
	if (erpMOD)
	{
		if (RS232_OpenComport(cport_nr, bdrate, mode, 0))
		{
			printf("RS232  : Can not open comport\n");
			return(0);
		}
	}
}

void Command_ERP42(void) {
    /*======================== SPEED & STEER ==========================*/
    // !! Please CHECK the Speed & Steer MODE values !!

    // SPEED RANGE  : Decimal [0 ~ 200] / HexaDecimal [0x00 ~ 0xc8]
    // Actual Speed : SPEED / 10 [KPH]



    if (vel_cmd < SLOW_SPEED) { // Fin CMD == 5
        speedcmd = vel_cmd;
    }
    else {
        speedcmd = vel_cmd - (vel_cmd - SLOW_SPEED) * fabs(delta_f) / 28.0;
    }


    if (SpeedMODE) {
        //writebuffer[eSPEED0] = (speedcmd &0xff00)>>8;            // Decimal 2 Hexa
        writebuffer[eSPEED1] = (speedcmd);// &0xff);            // Decimal 2 Hexa
        writebuffer[eGEAR] = 0x00;      //건들지 마시오
    }

    // STEER RANGE : Decimal [-2000 ~2000] / HexaDecimal [ ~ ]
    // Actual Steer : STEER / 71 [Degree]
    if (SteerMODE)
    {
        //CMD_delta_f();
        
        steercmd = -1 * 71 * delta_f;   // Degree 2 Decimal
        writebuffer[eSTEER0] = (steercmd & 0xff00) >> 8;   // Decimal 2 Hexa
        writebuffer[eSTEER1] = (steercmd & 0xff);   // Decimal 2 Hexa
    }

    if (parkingMOD)
    {
        if (vehicle_gear == Reverse) {
            writebuffer[eGEAR] = (GEAR_BACKWORD);
        }
        else if (vehicle_gear == Neutral) {
            writebuffer[eGEAR] = (GEAR_NEUTRAL);
        }
        else {
            writebuffer[eGEAR] = (GEAR_FOWARD);
        }
    }

    /*========================== write data ===========================*/
    write_len = RS232_SendBuf(cport_nr, &writebuffer, sizeof(writebuffer));
    writebuffer[eALIVE] = idx_alive; // loop마다 1씩 증가해서 오류 잡기 alive
    idx_alive++; idx_alive %= 0xff;   // 최대 255까지만 해서 버퍼 오버플로우 안나게
    idx_ndata++;
}

void Read_ERP42() // HZ 획인하기
{

    //ERPREAD
    read_len = RS232_PollComport(cport_nr, readbuffer, sizeof(readbuffer));
    if (read_len > 0)
        readbuffer[nREAD - 1] = 0;

    vel_read = readbuffer[7] / 10;          //<- 속도값 [km/h]

    // 16진수 -> 10진수 변환 코드
    if (readbuffer[9] > 245) // write가 양수면 read는 음수
        steer_read = (double)(-(255 - readbuffer[9] + 1) * pow(16, 2) + readbuffer[8]) / 71;
    else
        steer_read = (double)(readbuffer[9] * pow(16, 2) + readbuffer[8]) / 71;

    steer_read = -steer_read;                   //<- 조향각도값 [deg]

    buf_velRead  [count] = vel_read;
    buf_steerRead[count] = steer_read;

}

void CMD_delta_f()
{
    if (delta_f > MAX_STEER)
        delta_f = MAX_STEER;
    else if (delta_f < -MAX_STEER)
        delta_f = -MAX_STEER;
}

void CMD_Guidance2erp(int _spdcmd, int _brkcmd)
{
    CMD_delta_f();

    if (idx_way == idx_max[0])
    {
        speedcmd = 0;
        writebuffer[eBRAKE] = 50;
        //exitOuterLoop = true;
    }
    else
    {
        speedcmd = _spdcmd;
        writebuffer[eBRAKE] = _brkcmd;
        //printf("command OK\n");
    }

    if (fmod(simul_cnt, 2 == 0)) {
        Command_ERP42();
    }
}

void SaveData_ERP()
{
    char Filename[20];// = "_ERPdata.csv";
    strcpy(Filename, "_ERPdata.csv");

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
        fprintf(dataFile, "%.7f, %.7f\n",
            (double)buf_velRead[r], (double)buf_steerRead[r]
        );
    }

    printf("Save UDP flag Successful.\n");

    fclose(dataFile);
}


#endif // !ERP_COMMAND_C
