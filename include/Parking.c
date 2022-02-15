#ifndef MISSION
#define MISSION

#include "Parking.h"

void SelectParkingPosition()
{
	// For K-city
	//if (1 <= CHOOSE_PARKING_IDX && CHOOSE_PARKING_IDX <= 3)
	//{
	//	idx_park = park0[0];
	//}
	//else if (4 <= CHOOSE_PARKING_IDX && CHOOSE_PARKING_IDX <= 6)
	//{
	//	idx_park = park0[1];
	//}
	//else if (7 <= CHOOSE_PARKING_IDX && CHOOSE_PARKING_IDX <= 9)
	//{
	//	idx_park = park0[2];
	//}
	// 
	// 
	// For HANDONG


	if (1 <= CHOOSE_PARKING_IDX && CHOOSE_PARKING_IDX <= 4)
	{
		idx_park = park0[0]; //Start_point; // 
	}
	else if (5 <= CHOOSE_PARKING_IDX && CHOOSE_PARKING_IDX <= 7)
	{
		idx_park =park0[1]; //Start_point;// 
	}
	else if (8 <= CHOOSE_PARKING_IDX && CHOOSE_PARKING_IDX <= 10)
	{
		idx_park = park0[2]; //Start_point;// 
	}
	else if (11 <= CHOOSE_PARKING_IDX && CHOOSE_PARKING_IDX <= 13)
	{
		idx_park = park0[3]; //Start_point;// 
	}
		// 처음부터 Parking 을 시작할 경우 필요 없음. 
}

void ParkingMission()
{
	int case_center = 0;
	int case_center_1 = 0;
	int case_center_2 = 0;
	int case_center_3 = 0;
	int case_center_4 = 0;
	switch (PARKINGMODE) {

	case PARKING_INIT:

		vel_cmd = 0;
		brk_cmd = 20;

		PARK_X = buf_ParkInitX[CHOOSE_PARKING_IDX - 1] * LON2METER;
		PARK_Y = buf_ParkInitY[CHOOSE_PARKING_IDX - 1] * LAT2METER;
		PARK_ATT =(90.0 -  buf_ParkInitAtt[CHOOSE_PARKING_IDX - 1]) * DEG2RAD;
		//printf("PARK_X: %f\t PARK_Y: %f\t PARK_ATT:%f\n", PARK_X, PARK_Y, PARK_ATT);
		//printf("PARKING :: LON: %f\t LAT:%f\n ", buf_ParkInitX[CHOOSE_PARKING_IDX - 1], buf_ParkInitY[CHOOSE_PARKING_IDX - 1]);
		if (PARK_ATT > 180 * DEG2RAD) {
			PARK_ATT = PARK_ATT - 360 * DEG2RAD;
		}
		else if (PARK_ATT < -180 * DEG2RAD) {
			PARK_ATT = PARK_ATT + 360 * DEG2RAD;
		}


		parkInit_X = lon_filtered;
		parkInit_Y = lat_filtered;
		parkInit_ATT = yaw;
		//printf("LON: %f\t LAT: %f\n", parkInit_X, parkInit_Y);

		ParkInit_X_SUM = ParkInit_X_SUM + parkInit_X;
		ParkInit_Y_SUM = ParkInit_Y_SUM + parkInit_Y;
		ParkInit_ATT_SUM = ParkInit_ATT_SUM + parkInit_ATT;

		ParkInit_idx = ParkInit_idx + 1;

		if (ParkInit_idx == NPARKINIT) {
			PARK_INIT_X = ParkInit_X_SUM / NPARKINIT;        // [deg]
			PARK_INIT_Y = ParkInit_Y_SUM / NPARKINIT;        // [deg]
			//PARK_INIT_ATT = ParkInit_ATT_SUM / NPARKINIT;    // [rad]
			PARK_INIT_ATT = HWASUNG_YAW; //[deg]
			 ///////////////////////////////////////////////////////////////////////////////
			if (PARK_INIT_ATT > 180.0) {
				PARK_INIT_ATT = PARK_INIT_ATT - 360.0;
			}
			else if (PARK_INIT_ATT < -180.0) {
				PARK_INIT_ATT = PARK_INIT_ATT + 360.0;
			}
			else {
				PARK_INIT_ATT = PARK_INIT_ATT;
			}


			PARKINGMODE = TRAJECTORY;
			//printf("INIT_LOT: %f\t INIT_LAT: %f\n INIT_ATT: %f\n", PARK_INIT_X, PARK_INIT_Y, PARK_INIT_ATT);
		}

		break;

	case TRAJECTORY:

		vel_cmd = 0;
		brk_cmd = 1;

		CAR_X0 = PARK_INIT_X * LON2METER; //현재차량 위치와 자세
		CAR_Y0 = PARK_INIT_Y * LAT2METER;
		CAR_ATT0 = (90.0 - PARK_INIT_ATT) * DEG2RAD;
		// PARK_X, PARK_Y, PARK_ATT은 상황에 따라 정해짐 (TM좌표계, PARK_ATT = (90.0 - 센서값)*DEG2RAD

		if (CAR_ATT0 > 180 * DEG2RAD) {
			CAR_ATT0 = CAR_ATT0 - 360 * DEG2RAD;
		}
		else if (CAR_ATT0 < -180 * DEG2RAD) {
			CAR_ATT0 = CAR_ATT0 + 360 * DEG2RAD;
		}
		if (PARK_ATT > 180 * DEG2RAD)
			PARK_ATT = PARK_ATT - 360 * DEG2RAD;
		else if (PARK_ATT < -180 * DEG2RAD) {
			PARK_ATT = PARK_ATT + 360 * DEG2RAD;
		}

		SLOPE_init = tan(CAR_ATT0);
		SLOPE_goal = tan(PARK_ATT);

		intY_init = CAR_Y0 - SLOPE_init * CAR_X0;
		intY_goal = PARK_Y - SLOPE_goal * PARK_X;

		OSET_init = R_PARK * sqrt(SLOPE_init * SLOPE_init + 1);
		OSET_goal = R_PARK * sqrt(SLOPE_goal * SLOPE_goal + 1);

		intY_init_1 = intY_init + OSET_init;
		intY_init_2 = intY_init - OSET_init;
		intY_goal_1 = intY_goal + OSET_goal;
		intY_goal_2 = intY_goal - OSET_goal;

		X_1 = -(intY_init_1 - intY_goal_1) / (SLOPE_init - SLOPE_goal);
		Y_1 = (SLOPE_init * intY_goal_1 - SLOPE_goal * intY_init_1) / (SLOPE_init - SLOPE_goal);

		X_2 = -(intY_init_1 - intY_goal_2) / (SLOPE_init - SLOPE_goal);
		Y_2 = (SLOPE_init * intY_goal_2 - SLOPE_goal * intY_init_1) / (SLOPE_init - SLOPE_goal);

		X_3 = -(intY_init_2 - intY_goal_1) / (SLOPE_init - SLOPE_goal);
		Y_3 = (SLOPE_init * intY_goal_1 - SLOPE_goal * intY_init_2) / (SLOPE_init - SLOPE_goal);

		X_4 = -(intY_init_2 - intY_goal_2) / (SLOPE_init - SLOPE_goal);
		Y_4 = (SLOPE_init * intY_goal_2 - SLOPE_goal * intY_init_2) / (SLOPE_init - SLOPE_goal);

		X_mean = (CAR_X0 + PARK_X) / 2;
		Y_mean = (CAR_Y0 + PARK_Y) / 2;

		/*DIS_1 = sqrt((X_1 - X_mean) * (X_1 - X_mean) + (Y_1 - Y_mean) * (Y_1 - Y_mean));
		DIS_2 = sqrt((X_2 - X_mean) * (X_2 - X_mean) + (Y_2 - Y_mean) * (Y_2 - Y_mean));
		DIS_3 = sqrt((X_3 - X_mean) * (X_3 - X_mean) + (Y_3 - Y_mean) * (Y_3 - Y_mean));
		DIS_4 = sqrt((X_4 - X_mean) * (X_4 - X_mean) + (Y_4 - Y_mean) * (Y_4 - Y_mean));

		if (DIS_1 <= DIS_2)
		{
			DIS_CUR = DIS_1;
			X_one = X_1;
			Y_one = Y_1;
		}
		else
		{
			DIS_CUR = DIS_2;
			X_one = X_2;
			Y_one = Y_2;
		}

		if (DIS_3 <= DIS_CUR) {
			DIS_CUR = DIS_3;
			X_one = X_3;
			Y_one = Y_3;
		}
		if (DIS_4 <= DIS_CUR) {
			DIS_CUR = DIS_4;
			X_one = X_4;
			Y_one = Y_4;
		}*/
		if (Y_mean < (SLOPE_init * X_mean + intY_init))
		{
			if (Y_mean < (SLOPE_goal * X_mean + intY_goal))
			{
				case_center = 1;
			}
			else
			{
				case_center = 2;
			}
		}
		else
		{
			if (Y_mean < (SLOPE_goal * X_mean + intY_goal))
			{
				case_center = 3;
			}
			else
			{
				case_center = 4;
			}
		}
		if (Y_1 < (SLOPE_init * X_1 + intY_init))
		{
			if (Y_1 < (SLOPE_goal * X_1 + intY_goal))
			{
				case_center_1 = 1;
			}
			else
			{
				case_center_1 = 2;
			}
		}
		else
		{
			if (Y_1 < (SLOPE_goal * X_1 + intY_goal))
			{
				case_center_1 = 3;
			}
			else
			{
				case_center_1 = 4;
			}
		}

		if (Y_2 < (SLOPE_init * X_2 + intY_init))
		{
			if (Y_2 < (SLOPE_goal * X_2 + intY_goal))
			{
				case_center_2 = 1;
			}
			else
			{
				case_center_2 = 2;
			}
		}
		else
		{
			if (Y_2 < (SLOPE_goal * X_2 + intY_goal))
			{
				case_center_2 = 3;
			}
			else
			{
				case_center_2 = 4;
			}
		}

		if (Y_3 < (SLOPE_init * X_3 + intY_init))
		{
			if (Y_3 < (SLOPE_goal * X_3 + intY_goal))
			{
				case_center_3 = 1;
			}
			else
			{
				case_center_3 = 2;
			}
		}
		else
		{
			if (Y_3 < (SLOPE_goal * X_3 + intY_goal))
			{
				case_center_3 = 3;
			}
			else
			{
				case_center_3 = 4;
			}
		}

		if (Y_4 < (SLOPE_init * X_4 + intY_init))
		{
			if (Y_4 < (SLOPE_goal * X_4 + intY_goal))
			{
				case_center_4 = 1;
			}
			else
			{
				case_center_4 = 2;
			}
		}
		else
		{
			if (Y_4 < (SLOPE_goal * X_4 + intY_goal))
			{
				case_center_4 = 3;
			}
			else
			{
				case_center_4 = 4;
			}
		}

		if (case_center == case_center_1)
		{
			X_one = X_1;
			Y_one = Y_1;
		}
		else if (case_center == case_center_2)
		{
			X_one = X_2;
			Y_one = Y_2;
		}
		else if (case_center == case_center_3)
		{
			X_one = X_3;
			Y_one = Y_3;
		}
		else if (case_center == case_center_4)
		{
			X_one = X_4;
			Y_one = Y_4;
		}
		angle_delta = PARK_ATT - CAR_ATT0;
		if (angle_delta > 180 * DEG2RAD)
		{
			angle_delta = angle_delta - 360 * DEG2RAD;
		}
		else if (angle_delta < -180 * DEG2RAD) {
			angle_delta = angle_delta + 360 * DEG2RAD;
		}
		if (angle_delta <= 0)
		{
			for (size_t pkidx = 0; pkidx < (N_points_line_init); pkidx++)
			{
				buf_X_ParkingWP[pkidx] = CAR_X0 + ((double)(pkidx) + 1) / N_points_line_init * (X_one + R_PARK * cos(CAR_ATT0 + 90 * DEG2RAD) - CAR_X0);
				buf_Y_ParkingWP[pkidx] = CAR_Y0 + ((double)(pkidx) + 1) / N_points_line_init * (Y_one + R_PARK * sin(CAR_ATT0 + 90 * DEG2RAD) - CAR_Y0);
				buf_ATT_ParkingWP[pkidx] = CAR_ATT0;
			}
			for (size_t pkidx = N_points_line_init; pkidx < (N_points_line_init + N_points_one); pkidx++)
			{
				buf_X_ParkingWP[pkidx] = X_one + R_PARK * cos(CAR_ATT0 + 90 * DEG2RAD + ((double)(pkidx) - N_points_line_init) / N_points_one * angle_delta);
				buf_Y_ParkingWP[pkidx] = Y_one + R_PARK * sin(CAR_ATT0 + 90 * DEG2RAD + ((double)(pkidx) - N_points_line_init) / N_points_one * angle_delta);
				buf_ATT_ParkingWP[pkidx] = CAR_ATT0 + ((double)(pkidx) - N_points_line_init + 1) / N_points_one * (angle_delta);
			}
			for (size_t pkidx = N_points_line_init + N_points_one; pkidx < (N_points_line_init + N_points_one + N_points_line_goal); pkidx++)
			{
				buf_X_ParkingWP[pkidx] = X_one + R_PARK * cos(PARK_ATT + 90 * DEG2RAD) + ((double)(pkidx) - N_points_line_init - N_points_one + 1) / N_points_line_goal * (PARK_X - X_one - R_PARK * cos(PARK_ATT + 90 * DEG2RAD));
				buf_Y_ParkingWP[pkidx] = Y_one + R_PARK * sin(PARK_ATT + 90 * DEG2RAD) + ((double)(pkidx) - N_points_line_init - N_points_one + 1) / N_points_line_goal * (PARK_Y - Y_one - R_PARK * sin(PARK_ATT + 90 * DEG2RAD));
				buf_ATT_ParkingWP[pkidx] = PARK_ATT;
			}
		}
		else
		{
			for (size_t pkidx = 0; pkidx < (N_points_line_init); pkidx++)
			{
				buf_X_ParkingWP[pkidx] = CAR_X0 + ((double)(pkidx) + 1) / N_points_line_init * (X_one + R_PARK * cos(CAR_ATT0 - 90 * DEG2RAD) - CAR_X0);
				buf_Y_ParkingWP[pkidx] = CAR_Y0 + ((double)(pkidx) + 1) / N_points_line_init * (Y_one + R_PARK * sin(CAR_ATT0 - 90 * DEG2RAD) - CAR_Y0);
				buf_ATT_ParkingWP[pkidx] = CAR_ATT0;
			}
			for (size_t pkidx = N_points_line_init; pkidx < (N_points_line_init + N_points_one); pkidx++)
			{
				buf_X_ParkingWP[pkidx] = X_one + R_PARK * cos(CAR_ATT0 - 90 * DEG2RAD + ((double)(pkidx) - N_points_line_init + 1) / N_points_one * angle_delta);
				buf_Y_ParkingWP[pkidx] = Y_one + R_PARK * sin(CAR_ATT0 - 90 * DEG2RAD + ((double)(pkidx) - N_points_line_init + 1) / N_points_one * angle_delta);
				buf_ATT_ParkingWP[pkidx] = CAR_ATT0 + ((double)(pkidx) - N_points_line_init + 1) / N_points_one * (angle_delta);
			}
			for (size_t pkidx = N_points_line_init + N_points_one; pkidx < (N_points_line_init + N_points_one + N_points_line_goal); pkidx++)
			{
				buf_X_ParkingWP[pkidx] = X_one + R_PARK * cos(PARK_ATT - 90 * DEG2RAD) + ((double)(pkidx) - N_points_line_init - N_points_one + 1) / N_points_line_goal * (PARK_X - X_one - R_PARK * cos(PARK_ATT - 90 * DEG2RAD));
				buf_Y_ParkingWP[pkidx] = Y_one + R_PARK * sin(PARK_ATT - 90 * DEG2RAD) + ((double)(pkidx) - N_points_line_init - N_points_one + 1) / N_points_line_goal * (PARK_Y - Y_one - R_PARK * sin(PARK_ATT - 90 * DEG2RAD));
				buf_ATT_ParkingWP[pkidx] = PARK_ATT;
			}
		}

		PARKINGMODE = KANAYAMA;

		//if (parkWP_idx > NParkingWaypoint) {
		//	PARKINGMODE = KANAYAMA;
		//}

		//parkWP_idx = parkWP_idx + 1;

		break;

	case KANAYAMA:

		if (yaw > 180.0) {
			yaw = yaw - 360.0;
		}
		else if (yaw < -180.0) {
			yaw = yaw + 360.0;
		}
		else {
			yaw = yaw;
		}

		CAR_X = lon_filtered * LON2METER;
		CAR_Y = lat_filtered * LAT2METER;
		CAR_ATT = (90.0 - yaw) * DEG2RAD;
		//CAR_ATT = yaw * DEG2RAD;

		Dist_XtoWP = buf_X_ParkingWP[parkNwp_idx] - CAR_X;
		Dist_YtoWP = buf_Y_ParkingWP[parkNwp_idx] - CAR_Y;
		parkDist = sqrt(pow(Dist_XtoWP, 2) + pow(Dist_YtoWP, 2));
		//printf("DIST: %f\n", parkDist);
		//printf("LON: %f\t LAT: %f\n", lon_filtered, lat_filtered);
		buf_parkDist[kana_idx] = parkDist;

		if (CAR_ATT > 180.0 * DEG2RAD) {
			CAR_ATT = CAR_ATT - 360.0 * DEG2RAD;
		}
		else if (CAR_ATT < -180.0 * DEG2RAD) {
			CAR_ATT = CAR_ATT + 360.0 * DEG2RAD;
		}
		else {
			CAR_ATT = CAR_ATT;
		}

		if (CAR_ATT > 180.0 * DEG2RAD) {
			CAR_ATT = CAR_ATT - 360.0 * DEG2RAD;
		}
		else if (CAR_ATT < -180.0 * DEG2RAD) {
			CAR_ATT = CAR_ATT + 360.0 * DEG2RAD;
		}
		else {
			CAR_ATT = CAR_ATT;
		}

		Pc[0] = CAR_X;
		Pc[1] = CAR_Y;
		Pc[2] = CAR_ATT;

		Pr[0] = buf_X_ParkingWP[parkNwp_idx];
		Pr[1] = buf_Y_ParkingWP[parkNwp_idx];
		Pr[2] = buf_ATT_ParkingWP[parkNwp_idx];

		Te[0][0] = cos(CAR_ATT); Te[0][1] = sin(CAR_ATT); Te[0][2] = 0.0;
		Te[1][0] = -sin(CAR_ATT); Te[1][1] = cos(CAR_ATT); Te[1][2] = 0.0;
		Te[2][0] = 0.0; Te[2][1] = 0.0; Te[2][2] = 1.0;

		Pe[0] = Te[0][0] * (Pr[0] - Pc[0]) + Te[0][1] * (Pr[1] - Pc[1]) + Te[0][2] * (Pr[2] - Pc[2]);
		Pe[1] = Te[1][0] * (Pr[0] - Pc[0]) + Te[1][1] * (Pr[1] - Pc[1]) + Te[1][2] * (Pr[2] - Pc[2]);
		Pe[2] = Te[2][0] * (Pr[0] - Pc[0]) + Te[2][1] * (Pr[1] - Pc[1]) + Te[2][2] * (Pr[2] - Pc[2]);

		Parking_Vel_cmd = PARK_VEL_REF * cos(Pe[2]) + K_PARKX * Pe[0];
		buf_parking_vel_cmd[kana_idx] = Parking_Vel_cmd;

		Parking_w_cmd = PARK_W_REF + PARK_VEL_REF * (K_PARKY * Pe[1]) + K_PARKTH * sin(Pe[2]);  
		buf_parking_w_cmd[kana_idx] = Parking_w_cmd;

		Parking_delta_des = atan2(Parking_w_cmd * BODY_L, Parking_Vel_cmd);

		Kana_angle_error = buf_ATT_ParkingWP[parkNwp_idx] - CAR_ATT;        
		printf("Kanayama angle error: %f\n", Kana_angle_error * RAD2DEG);
		buf_Kana_Angle_Error[kana_idx] = Kana_angle_error;

		//if (Parking_delta_des > MAX_STEER * DEG2RAD)
		//	Parking_delta_des = MAX_STEER * DEG2RAD;
		//else if (Parking_delta_des < -MAX_STEER * DEG2RAD)
		//	Parking_delta_des = -MAX_STEER * DEG2RAD;
		//else
		//	Parking_delta_des = Parking_delta_des;

		if (Parking_Vel_cmd < 0.0) {
			Parking_Vel_cmd = abs(Parking_Vel_cmd);
		}
		else {
			Parking_Vel_cmd = Parking_Vel_cmd;
		}

		// PARAMETERS TO CHANGE COMMAND NEAR X_STAR,Y_STAR
		gain_delta_f = 1.0;
		MIN_PARKDIS = 1.8;

		//// TUNING
		//if (parkNwp_idx <= star_WP_idx) {
		//	gain_delta_f = 3.0;
		//	MIN_PARKDIS = 2.5;
		//}
		//else if (parkNwp_idx == star_WP_idx + 1)
		//{
		//	gain_delta_f = 10.0;
		//	MIN_PARKDIS = 2.0;
		//}
		//else if (parkNwp_idx == star_WP_idx + 2)
		//{
		//	gain_delta_f = 10.0;
		//	MIN_PARKDIS = 1.8;
		//}
		//else if (parkNwp_idx == star_WP_idx + 3)
		//{
		//	gain_delta_f = 10.0;
		//	MIN_PARKDIS = 1.6;
		//}
		//else if (parkNwp_idx == star_WP_idx + 4)
		//{
		//	gain_delta_f = 7.0;
		//	MIN_PARKDIS = 1.3;
		//}
		//else if (parkNwp_idx == star_WP_idx + 5)
		//{
		//	gain_delta_f = 5.0;
		//	MIN_PARKDIS = 1.0;
		//}
		//else if (parkNwp_idx == star_WP_idx + 6)
		//{
		//	gain_delta_f = 4.0;
		//	MIN_PARKDIS = 0.8;
		//}
		//else {
		//	gain_delta_f = 3.0;
		//	MIN_PARKDIS = 0.6;
		//}


		// INDEX UPDATE ACCORDING TO DISTANCE FROM WAYPOINT
		printf("WAYPOINT IDX: %d\n", parkNwp_idx);
		if (parkNwp_idx < NParkingWaypoint) {
			if (parkDist < MIN_PARKDIS) {
				parkNwp_idx = parkNwp_idx + 1;
			}
			else {
				parkNwp_idx = parkNwp_idx;
			}
		}
		else {
			PARKINGMODE = PARKING_STOP;
		}


		buf_parking_delta_des[kana_idx] = Parking_delta_des;

		delta_f = Parking_delta_des * RAD2DEG;
		//=======================================================
		delta_f = gain_delta_f * delta_f;
		//=======================================================
		vel_cmd = Parking_Vel_cmd;

		kana_idx = kana_idx + 1;

		break;

	case PARKING_STOP:

		parking_stop_idx = parking_stop_idx + 1;

		if (parking_stop_idx < 100) {   // 5초 정지
			vel_cmd = 0;
			brk_cmd = 50;
			printf("PARKING STOP: %d\n", brk_cmd);
		}
		else {
			PARKINGMODE = BACKOUT;
		}

		break;

	case BACKOUT:

		brk_cmd = 1;

		CAR_X = lon_filtered * LON2METER;
		CAR_Y = lat_filtered * LAT2METER;

		Back_DistX = CAR_X - PARK_X;
		Back_DistY = CAR_Y - PARK_Y;
		Back_Dist = sqrt(pow(Back_DistX, 2) + pow(Back_DistY, 2));

		vehicle_gear = Reverse; // reverse
		//vel_cmd = buf_parking_vel_cmd[kana_idx - back_idx];
		vel_cmd = 80.0;

		if (Back_Dist > BACK_MIN_DIST) {
			//vehicle_gear = neutral; or
			vel_cmd = 0;
			PARKINGMODE = STOP_TO_GO;

		}
		else {
			delta_f = 0.0;
		}

		back_idx = back_idx + 1;

		buf_backX[park_idx] = CAR_X;
		buf_backY[park_idx] = CAR_Y;

		park_idx = park_idx + 1;

		break;

	case STOP_TO_GO:

		if (stop_idx < 30) {
			vehicle_gear = Drive;
			// vehicle_gear = Neutral;
			vel_cmd = 0;
			brk_cmd = 50;

		}

		stop_idx = stop_idx + 1;

		break;

	default:

		printf("YOU ARE IN THE DEFAULT MODE\n");

		break;

	}
	printf("PARKINGMODE: %d\n", PARKINGMODE);
	buf_parking_Xcarpath[count] = lon_filtered;
	buf_parking_Ycarpath[count] = lat_filtered;
	buf_parking_psi[count] = (90.0 - yaw);
	buf_parking_delta_f[count] = delta_f;
	buf_parking_vel_cmd[count] = vel_cmd;

}


void SaveData_Parking()
{
	char Filename[20];// = "_ParkingData.csv";
	strcpy(Filename, "_ParkingData.csv");

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
		fprintf(dataFile, "%.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f\n",
			(double)buf_X_ParkingWP[r], (double)buf_Y_ParkingWP[r], (double)buf_ATT_ParkingWP[r],
			(double)buf_parking_psi[r], (double)buf_parking_vel_cmd[r], (double)buf_parking_w_cmd[r], (double)buf_parking_delta_des[r],
			(double)buf_parking_Xcarpath[r], (double)buf_parking_Ycarpath[r], (double)buf_parking_delta_f[r], (double)buf_parking_vel_cmd[r], (double)buf_Star_XY[r], (double)buf_parkDist[r]);
	}

	printf("Parking Guidance Data Successful.\n");

	fclose(dataFile);

	printf("Parking Data Successful.\n");

}

#endif // !MISSION
