#ifndef FLAG_C
#define FLAG_C

#include "Flag.h"
void TrafficCmd();
void DeliveryCmd();
void UnptCmd();
void LeftCmd();

int prev_Traffic_flag = 18;
int count_A1 = 0;
int count_A2 = 0;
int count_A3 = 0;
int Main_del_number = 0;



//============================//
//         MOD select         //
//============================//
void Mission_select(int m_park, int m_obs, int m_sign, int m_del)
{
	mission_park = m_park;
	mission_obs  = m_obs ;
	mission_sign = m_sign;
	mission_del  = m_del ;
}

void MOD_select(int _xsens, int _novatel, int _lidar, int _camObject, int _camLine, int _global, int _local, int _erp, int _parking, int _cluster, int _deliver)
{
	xsensMOD = _xsens;
	novatelMOD = _novatel;
	lidarMOD = _lidar;
	camobMOD = _camObject;
	camlaneMOD = _camLine;
	globalMOD = _global;
	localMOD = _local;
	erpMOD = _erp;
	parkingMOD = _parking;
	clusterMOD = _cluster;
	deliveryMOD = _deliver;
}

//============================//
//       MISSONG Flag         //
//============================//
void Mission_Flag()
{	
	// ============== Parking Flag =================== //

	//printf("idx_park: %d\n", idx_park);
	if (mission_park)
	{
		if (idx_way == idx_park)
		{
			parkingMOD = Parking_ON;
			globalMOD = Global_OFF;
			localMOD = Local_OFF;
		}
		else
			parkingMOD = Parking_OFF;
	}


	// ============== Obstacle Flag ======================//
	if (mission_obs)
	{
		if (OBS_WP[0] <= idx_way && idx_way <= OBS_WP[1])
		{
			obstacleMOD = Obstacle_ON;
		}
		else
		{
			obstacleMOD = Obstacle_OFF;
		}
	}


	// ============== Traffic Sign Flag =================== //
	UNPT_LINE = STOP_LINE[8];
	LEFT_LINE = STOP_LINE[9];
	mission_sign = 0;
	//printf("signCMD: %d\n////////", mission_sign);
	for (int c = 0; c < idx_sign[0]; c++)
	{
		if (idx_way == UNPT_LINE)// || idx_way == UNPT_LINE + 1)
		{
			mission_sign = 1;
		}
		else if (idx_way == LEFT_LINE)
		{
			mission_sign = 1;
		}
		else if (idx_way == STOP_LINE[c])
		{
			mission_sign = 1;
		}
	}
	
	if (mission_sign)
	{
		for (int c = 0; c < idx_sign[0]; c++)
		{
			if (idx_way == UNPT_LINE)// || idx_way == UNPT_LINE + 1)
			{
				//printf("UNPT_LINE : ON\n");
				Traffic_flag = RecvArray_trafficSign[0];
				ERP_flag = DO_Unpt_Command;
			}
			else if (idx_way == LEFT_LINE)
			{
				//printf("LEFT_LINE : ON\n");
				Traffic_flag = RecvArray_trafficSign[0];
				ERP_flag = DO_Left_Command;
			}
			else if (idx_way == STOP_LINE[c])
			{
				//printf("STOP_LINE : ON\n");
				Traffic_flag = RecvArray_trafficSign[0];
				ERP_flag = DO_Traffic_Command;
			}
		}
	}
	else
	{
		prev_Traffic_flag = 18;
	}

	// ============== Delivery Flag =================== //
	if (mission_del)
	{
		if (idx_way == (DeliveryIDX[DelA][StartIDX] - 1 ))//&& idx_way <= DeliveryIDX[DelA][FinalIDX])
		{
			deliveryMOD = Delivery_ON;
			ERP_flag = DO_DeliveryA_Command;
		}
		else if (idx_way == (DeliveryIDX[DelB][StartIDX] - 1 ))// && idx_way <= DeliveryIDX[DelB][FinalIDX])
		{
			deliveryMOD = Delivery_ON;
			ERP_flag = DO_DeliveryB_Command;
		}
	}
}

void Mission_Command()
{
	//printf("MIssion cmd\n");

	if (mission_sign)
	{
		if (ERP_flag == DO_Traffic_Command)
		{
			vel_cmd = MAX_SPEED;
			brk_cmd = 1;
			TrafficCmd();
		}

		if (ERP_flag == DO_Left_Command)
		{
			vel_cmd = MAX_SPEED;
			brk_cmd = 1;
			LeftCmd();
		}

		if (ERP_flag == DO_Unpt_Command)
		{
			vel_cmd = MAX_SPEED;
			brk_cmd = 1;

			UnptCmd();
		}
	}

	if (deliveryMOD)
		DeliveryCmd();

}
void TrafficCmd()
{
	if (Traffic_flag == 18)
	{
		Traffic_flag = prev_Traffic_flag;
	}
	switch (Traffic_flag)
	{
	case FLAG_STOP:
		vel_cmd = STOP_SPEED;
		//brk_cmd = 1;
		brk_cmd = 20;
		break;
		//printf("TRK_FLAG : STOP MODE!!! \n\n");
	case FLAG_GO:
		vel_cmd = MAX_SPEED;
		brk_cmd = 1;
		break;
		//printf("TRK_FLAG : GO MODE!!! \n\n");
	case FLAG_LEFT:
		vel_cmd = STOP_SPEED;
		brk_cmd = 20;
		break;
		//printf("TRK_FLAG : LEFT MODE!!! \n\n");
	case FLAG_GOLEFT:
		vel_cmd = MAX_SPEED;
		brk_cmd = 1;
		break;
		//printf("TRK_FLAG : GOLEFT MODE!!! \n\n");
	case FLAG_ERROR:
		vel_cmd = STOP_SPEED;
		brk_cmd = 20;
		break;
		//printf("TRK_FLAG : SLOW MODE!!! \n\n");
	default:
		vel_cmd = MAX_SPEED;
		brk_cmd = 1;
		break;
		//printf("TRK_FLAG : DRFAULT MODE!!! \n\n");
	}
	prev_Traffic_flag = Traffic_flag;

}

void UnptCmd()
{
	if (Traffic_flag == 18)
	{
		Traffic_flag = prev_Traffic_flag;
	}
	switch (Traffic_flag)
	{
	case FLAG_STOP:
		vel_cmd = STOP_SPEED;
		//brk_cmd = 1;
		brk_cmd = 20;
		break;
		//printf("UNPT_FLAG : STOP MODE!!! \n\n");
	case FLAG_GO:
		vel_cmd = MAX_SPEED;
		brk_cmd = 1;
		break;
		//printf("UNPT_FLAG : GO MODE!!! \n\n");
	case FLAG_LEFT:
		vel_cmd = MAX_SPEED;
		brk_cmd = 1;
		break;
		//printf("UNPT_FLAG : LEFT MODE!!! \n\n");
	case FLAG_GOLEFT:
		vel_cmd = MAX_SPEED;
		brk_cmd = 1;
		break;
		//printf("UNPT_FLAG : GOLEFT MODE!!! \n\n");
	case FLAG_ERROR:
		vel_cmd = STOP_SPEED;
		brk_cmd = 20;
		break;
		//printf("UNPT_FLAG : SLOW MODE!!! \n\n");
	default:
		vel_cmd = MAX_SPEED;
		brk_cmd = 1;
		break;
		//printf("UNPT_FLAG : DRFAULT MODE!!! \n\n");
	}
	prev_Traffic_flag = Traffic_flag;

}

void LeftCmd()
{
	if (Traffic_flag == 18)
	{
		Traffic_flag = prev_Traffic_flag;
	}
	switch (Traffic_flag)
	{
	case FLAG_STOP:
		vel_cmd = STOP_SPEED;
		brk_cmd = 20;
		break;
		//printf("LEFT_FLAG : FLAG_STOP MODE!!! \n\n");
	case FLAG_GO:
		vel_cmd = STOP_SPEED;
		brk_cmd = 20;
		break;
		//printf("LEFT_FLAG : GO MODE!!! \n\n");
	case FLAG_LEFT:
		vel_cmd = MAX_SPEED;
		brk_cmd = 1;
		break;
		//printf("LEFT_FLAG : LEFT MODE!!! \n\n");
	case FLAG_GOLEFT:
		vel_cmd = MAX_SPEED;
		brk_cmd = 1;
		break;
		//printf("LEFT_FLAG : GOLEFT MODE!!! \n\n");
	case FLAG_ERROR:
		vel_cmd = STOP_SPEED;
		brk_cmd = 20;
		break;
		//printf("LEFT_FLAG : SLOW MODE!!! \n\n");
	default:
		vel_cmd = MAX_SPEED;
		brk_cmd = 1;
		break;
		//printf("LEFT_FLAG : DRFAULT MODE!!! \n\n");
	}
	prev_Traffic_flag = Traffic_flag;
}

void DeliveryCmd()
{
	//printf("DEL\n");
	// Delivery sign Flag from Camera
	buf_delA_flag1[count] = Delivery_flagA1;
	buf_delA_flag2[count] = Delivery_flagA2;
	buf_delA_flag3[count] = Delivery_flagA3;
	buf_delB_flag1[count] = Delivery_flagB1;
	buf_delB_flag2[count] = Delivery_flagB2;
	buf_delB_flag3[count] = Delivery_flagB3;

	switch (ERP_flag)
	{
	case DO_DeliveryA_Command:

		if (buf_signflag[2][count] == 6)
		{
			count_A1++;
		}
		else if (buf_signflag[3][count] == 7)
		{
			count_A2++;
		}
		else if (buf_signflag[4][count] == 8)
		{
			count_A3++;
		}

		if (count_A1 > count_A2)
		{
			if (count_A1 > count_A3)
			{
				Main_del_number = 1;
			}
			else
			{
				Main_del_number = 3;
			}
		}
		else
		{
			if (count_A2 > count_A3)
			{
				Main_del_number = 2;
			}
			else
			{
				Main_del_number = 3;
			}
		}
		if (mode_print_del == 1)
		{
			printf("Main_del_number = %d\n", Main_del_number);
		}
		//if ((buf_delA_flag1[count] == A1)|| (buf_delA_flag2[count] == A2) || (buf_delA_flag3[count] == A3))
		//{
		//	flagA_cnt = 0;
		//	delA_stopFlag = 0;
		//}
		if (buf_delA_flag1[count - 1] == A1 && buf_delA_flag1[count] == NO_FLAG ||
			buf_delA_flag2[count - 1] == A2 && buf_delA_flag2[count] == NO_FLAG ||
			buf_delA_flag3[count - 1] == A3 && buf_delA_flag3[count] == NO_FLAG)
		{
			delA_stopFlag = 1;
		}
		else {
			vel_cmd = MID_SPEED;
		}
		// Distance Calculation after Sign Detection Fail
		if (delA_stopFlag)
		{
			if (flagA_cnt == 0) {// extern is needed
				init_Pos_lat_DA = lat_filtered;
				init_Pos_lon_DA = lon_filtered;
			}
			else
			{
				delA_Vec_BodyToWay[0][0] = (lat_filtered - init_Pos_lat_DA) * LAT2METER;
				delA_Vec_BodyToWay[0][1] = (lon_filtered - init_Pos_lon_DA) * LON2METER;
				delA_Dist_R = sqrt(delA_Vec_BodyToWay[0][0] * delA_Vec_BodyToWay[0][0] + delA_Vec_BodyToWay[0][1] * delA_Vec_BodyToWay[0][1]);

				//printf("Distance : %f\n", delA_Dist_R);

				vel_cmd = SLOW_SPEED;

				if (delA_Dist_R > THRESH_DIST)
				{
					ERP_flag = FIN_DeliveryA_CMD;
					//Fin_cnt = 0;
					break;
				}
			}
			flagA_cnt++;

		}

		break;

	case DO_DeliveryB_Command:
		if (Main_del_number == 1)
		{
			if (buf_delB_flag1[count] == B1)
			{
				flagB_cnt = 0;
				delB_stopFlag = 0;
			}
			if (buf_delB_flag1[count - 1] == B1 && buf_delB_flag1[count] == NO_FLAG)
			{
				delB_stopFlag = 1;
			}
			else {
				vel_cmd = MID_SPEED;
				if (mode_print_del == 1)
				{
					printf("SLOW_B1\n");
				}
			
			}
		}
		else if (Main_del_number == 2)
		{
			if (buf_delB_flag2[count] == B2)
			{
				flagB_cnt = 0;
				delB_stopFlag = 0;
			}
			if (buf_delB_flag2[count - 1] == B2 && buf_delB_flag2[count] == NO_FLAG)
			{
				delB_stopFlag = 1;
			}
			else {
				vel_cmd = MID_SPEED;
				if (mode_print_del == 1)
				{
					printf("SLOW_B2\n");
				}
			}
		}
		else if (Main_del_number == 3)
		{
			if (buf_delB_flag3[count] == B3)
			{
				flagB_cnt = 0;
				delB_stopFlag = 0;
			}
			if (buf_delB_flag3[count - 1] == B3 && buf_delB_flag3[count] == NO_FLAG)
			{
				delB_stopFlag = 1;
			}
			else{
				vel_cmd = MID_SPEED;
				if (mode_print_del == 1)
				{
					printf("SLOW_B3\n");
				}
		}
	}
		//if (buf_delB_flag1[count - 1] == B1 && buf_delB_flag1[count] == NO_FLAG ||
		//	buf_delB_flag2[count - 1] == B2 && buf_delB_flag2[count] == NO_FLAG ||
		//	buf_delB_flag3[count - 1] == B3 && buf_delB_flag3[count] == NO_FLAG)
		//{
		//	delB_stopFlag = 1;
		//}
		//else
		//	vel_cmd = SLOW_SPEED;


		if (delB_stopFlag)
		{
			if (flagB_cnt == 0) {// extern is needed
				init_Pos_lat_DB = lat_filtered;
				init_Pos_lon_DB = lon_filtered;
			}
			else
			{
				delB_Vec_BodyToWay[0][0] = (lat_filtered - init_Pos_lat_DB) * LAT2METER;
				delB_Vec_BodyToWay[0][1] = (lon_filtered - init_Pos_lon_DB) * LON2METER;
				delB_Dist_R = sqrt(delB_Vec_BodyToWay[0][0] * delB_Vec_BodyToWay[0][0] + delB_Vec_BodyToWay[0][1] * delB_Vec_BodyToWay[0][1]);
				printf("Distance : %f\n", delB_Dist_R);
				vel_cmd = SLOW_SPEED;
				if (delB_Dist_R > THRESH_DIST)
				{
					ERP_flag = FIN_DeliveryB_CMD;
					break;
				}
			}
			flagB_cnt++;
		}

		break;

	case FIN_DeliveryA_CMD:
		//printf("Fin cnt %d", Fin_cnt);
		if (Fin_cnt < 120) { // 5 sec stop 
			vel_cmd = 0;
			brk_cmd = 50;
		}
		else {
			Fin_cnt = 0;
			vel_cmd = MAX_SPEED;
			brk_cmd = 1;
			deliveryMOD = Delivery_OFF;
			// Maybe default mode is more feasible Need a talk with JS Park
		}

		Fin_cnt++;
		break;

	case FIN_DeliveryB_CMD:
		//printf("Fin cnt %d", Fin_cnt);
		if (Fin_cnt < 120) { // 5 sec stop 
			vel_cmd = 0;
			brk_cmd = 50;
		} else {
			//Fin_cnt = 0;
			vel_cmd = MAX_SPEED;
			brk_cmd = 1;
			deliveryB_flag = 1;
			deliveryMOD = Delivery_OFF;
			// Maybe default mode is more feasible Need a talk with JS Park
		}

		Fin_cnt++;
		break;

	}
}
#endif // !FLAG_C
