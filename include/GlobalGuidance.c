#pragma once
#ifndef GLOBAL_GUIDANCE
#define GLOBAL_GUIDANCE


#include "GlobalGuidance.h"

void ReadWaypoint()
{
    FILE* waypoint;
    waypoint = fopen("way_point.csv", "r");  // waypoint 바꾸는 구역

    if (waypoint != NULL)
    {
        //printf("TEST");
        for (int c = 0; c < BUF_SIZE; c++) {
            fscanf(waypoint, "%d, %lf, %lf,", &idx_max[c], &GPS_waypoint_Lat[c], &GPS_waypoint_Lon[c]);
            //printf("LAT = %lf\tLON = %lf\n", GPS_waypoint_Lat[c], GPS_waypoint_Lon[c]);
        }
        fclose(waypoint);
        printf("Save waypoint...\n");
    }
    else
    {
        printf("There is no WAYPOINT file...\n");
        return 0;
    }

    FILE* waypointIDX;
    waypointIDX = fopen("MID_waypoint.csv", "r");

    if (waypointIDX != NULL)
    {
        fscanf(waypointIDX, "%d", &waypt_IDX);
        fclose(waypointIDX);
        printf("MID point waypoint: %d\n", waypt_IDX);
    }
    else
    {
        printf("MID waypoint IDX file is empty...\n");
    }


    FILE* xsensYaw;
    xsensYaw = fopen("YAW_Data.csv", "r");

    if (xsensYaw != NULL)
    {
        for (int c = 0; c < BUF_SIZE; c++)
            fscanf(xsensYaw, "%d, %lf, %lf, %lf,", &xsens_idx[c], &xsens_lat[c], &xsens_lon[c], &initialization_yaw[c]);

        //for (int c = 0; c < xsens_idx[0]; c++)
        //    printf("yaw data: %d\t%f\t%f\t%f\n", xsens_idx[c], xsens_lat[c], xsens_lon[c], initialization_yaw[c]);


        fclose(xsensYaw);
        printf("Save Initial Yaw data...\n");
    }
    else
    {
        printf("Yaw data is not found...\n");
    }
}

void SaveMissionPoint()
{
    FILE* signpoint;
    signpoint = fopen("sign_point.csv", "r");

    for (int c = 0; c < BUF_SIZE; c++)
        fscanf(signpoint, "%d, %d", &idx_sign[c], &STOP_LINE[c]);

    for (int c = 0; c < idx_sign[0]; c++)
        printf("sign point: %d\t%d\n", idx_sign[c], STOP_LINE[c]);

    
    
    
    
    

    fclose(signpoint);



    FILE* obstaclepoint;
    obstaclepoint = fopen("obstacle_point.csv", "r");

    for (int c = 0; c < 2; c++)
        fscanf(obstaclepoint, "%d, %d", &idx_obs[c], &OBS_WP[c]);


    for (int c = 0; c < idx_obs[0]; c++)
        printf("Obs point: %d\t%d\n", idx_obs[c], OBS_WP[c]);

    fclose(obstaclepoint);


    printf("Save Obstacle waypoint...\n");


    FILE* Delpoint;
    Delpoint = fopen("del_point.csv", "r");

    for (int c = 0; c < 2; c++)
        fscanf(Delpoint, "%d, %d, %d", &idx_del[c], &DeliveryIDX[DelA][c], &DeliveryIDX[DelB][c]);


    for (int c = 0; c < idx_del[0]; c++)
        printf("Del point: %d\t%d\t%d\n", idx_del[c], DeliveryIDX[DelA][c], DeliveryIDX[DelB][c]);

    fclose(Delpoint);

    //for (int c = 0; c < idx_delA[0]; c++)
    //    printf("%d, %d\n", idx_delA[c], DEL_A_WP[c]);

    printf("Save Delivery waypoint...\n");

}

void Save_MIDwaypoint()
{
    dataFile = fopen("MID_waypoint.csv", "wt");
    fprintf(dataFile, "%d\n", (int)idx_way);
    fclose(dataFile);
}

void SelectWaypoint()
{
    if (waypt_IDX > IDX_STANDARD)
    {
        init_detect  = IDX_STANDARD - 3;
        final_detect = idx_max[0];
    }
    else
    {
        init_detect  = 0;
        final_detect = IDX_STANDARD + 3;
    }
    printf("::::: IDX detection ::::: \n(Init point) %d\t(Final point)%d\n\n", init_detect, final_detect);

    for (int i = init_detect; i < final_detect; i++)
    {
        //cnt++;
        //printf("cn: %d\n", cnt);
        Way_dist = sqrt(pow(GPS_waypoint_Lat[i] * LAT2METER - LAT0 * LAT2METER, 2) + pow(GPS_waypoint_Lon[i] * LON2METER - LON0 * LON2METER, 2));
        if (Min_dist > Way_dist)
        {
            Min_dist = Way_dist;
            Start_point = i;

        }
    }
    //idx_max[0] = idx_max[0] - Start_point;
    //printf("idx_max:%d\n", idx_max[0]);

    Start_point++;
    GPS_waypoint_Lat_now = GPS_waypoint_Lat[Start_point];
    GPS_waypoint_Lon_now = GPS_waypoint_Lon[Start_point];
    idx_way = Start_point;

    printf("start point : %f\t%f\t%d\n", GPS_waypoint_Lat_now, GPS_waypoint_Lon_now, Start_point);
}

void Select_Xsens_yaw()
{
    for (int i = 0; i < xsens_idx[0]; i++)
    {
        Way_dist = sqrt(pow(xsens_lat[i] - LAT0, 2) + pow(xsens_lon[i] - LON0, 2));
        if (Min_dist > Way_dist)
        {
            Min_dist = Way_dist;
            Start_point = i;
        }
    }
    //Start_point++;
    Real_Yaw = initialization_yaw[Start_point];
}


void Xsens_Initialization()
{
    double yawyaw;
    printf("===========IMU Initialization Start!===========\n");

    Select_Xsens_yaw();
    //Real_Yaw = NEW2GAL;
    //////////////////////////////////////////////////////////////////////////
    Real_Yaw = HWASUNG_YAW;
    //////////////////////////////////////////////////////////////////////////
    Import_Xsens(); //teat park

    printf("Current YAW: %f\tREAL YAW: %f\n", yaw, Real_Yaw);
    //wait_time(2);
    InitTImeCheck();
    do
    {
        Import_Xsens();

        //printf("%f\n", yaw);
        yawyaw = Real_Yaw - yaw;
        yaw_buffer = yaw_buffer + yawyaw;
        Idle_Time();
    } while (simu_t < XSENS_INITIAL_TIME);

    yaw_OFFSET = yaw_buffer / simul_cnt;

    printf("YAW offset: %f\n\n", yaw_OFFSET);

}

void NovAtel_Initialization()
{
    printf("===========GPS Initialization Start!===========\n\n");

	printf("Wait RTK mode...\n");
	InitTImeCheck();
	do
	{
		Import_Novatel();
		Idle_Time();

	} while (fixqual != '4');

    if (fixqual == '4')
        printf("RTK mode successful!\n\n");

    do
    {
        printf("Check initial Position...\n");
        InitTImeCheck();
        do
        {
            Import_Novatel();
            if ((36.0 < latitude) && (latitude < 38.0) && (126.0 < longitude) && (longitude < 130))
            {
                Lat_init = Lat_init + latitude;
                Lon_init = Lon_init + longitude;

                cnt_initial++;
            }
            Idle_Time();
            //printf("check\n");

        } while (simu_t < POS_INITIAL_TIME);

        LAT0 = Lat_init / cnt_initial;
        LON0 = Lon_init / cnt_initial;

        if ((36.0 > LAT0) || (LAT0 > 39.0) || (126.0 > LON0) || (LON0 > 130) || cnt_initial == 0)
        {
            printf("Initialization Failed:(\n");
        }
    } while ((36.0 > LAT0) || (LAT0 > 39.0) || (126.0 > LON0) || (LON0 > 130.0) || cnt_initial == 0);


    //printf("Lat0: %f\tLon0: %f\n", LAT0, LON0);

   //GPS_waypoint_Lat_now = GPS_waypoint_Lat[0];
   //GPS_waypoint_Lon_now = GPS_waypoint_Lon[0];

    SelectWaypoint();

    printf("Initialization Succeed:D\n");
    eX_[0][0] = LAT0;
    eX_[2][0] = LON0;
}


void Pursuit_Guidance(void) {
    double best_delta = 0.0;

    gamma = 90.0 - yaw;//buf_Raw_Yaw[count];
    if (gamma > 180.0)
        gamma = gamma - 360.0;
    else if (gamma < -180.0)
        gamma = gamma + 360.0;

    gamma = gamma * DEG2RAD;


    // Distance and Lambda Calculation
    Vec_BodyToWay[0][0] = (GPS_waypoint_Lat_now - lat_filtered) * LAT2METER;
    Vec_BodyToWay[0][1] = (GPS_waypoint_Lon_now - lon_filtered) * LON2METER;

    Dist_R = sqrt(Vec_BodyToWay[0][0] * Vec_BodyToWay[0][0] + Vec_BodyToWay[0][1] * Vec_BodyToWay[0][1]);
    lambda = atan2(Vec_BodyToWay[0][0], Vec_BodyToWay[0][1]);

    // Delta Calculation
    delta = gamma - lambda;
    if (delta > 180 * DEG2RAD)
        delta = delta - 360 * DEG2RAD;
    else if (delta < -180 * DEG2RAD)
        delta = delta + 360 * DEG2RAD;
    /////////////////////////////////////////////////////////////////////
    //printf("delta = %f\n", delta);
    //printf("Dist_R = %f\n", Dist_R);

    /////////////////////////////////////////////////////////////////////
    // Delta_f Calculation
    if (delta >= 90 * DEG2RAD)
    {
        best_delta = 1;
    }
    else if (delta <= -90 * DEG2RAD)
    {
        best_delta = -1;
    }
    else
    {
        best_delta = sin(delta);
    }

    if (Dist_R < 5.0)
    {
        delta_f = (-2 * BODY_L * global_K * best_delta) / 5.0;
        delta_f = RAD2DEG * delta_f;
    }
    else {
        delta_f = (-2 * BODY_L * global_K * best_delta) / Dist_R;
        delta_f = RAD2DEG * delta_f;

    }

    //CMD_delta_f();

    buf_Index[count] = count;
    buf_delta[count] = delta;
    buf_lambda[count] = lambda;
    buf_Dist_R[count] = Dist_R;
    buf_gamma[count] = gamma;
    buf_delta_f[count] = delta_f;
    buf_waylatnow[count] = GPS_waypoint_Lat_now;
    buf_waylonnow[count] = GPS_waypoint_Lon_now;
}

void WayPoint_Change() {
    double LAT_OFFSET = 0.0;
    double LON_OFFSET = 0.0;
    double del_offsetM = 0.0;
    double del_pose = 0.0;

    del_offsetM = KCITY_DELDIST;
    del_pose    = KCITY_POSE;

    if (del_pose > 0)
        del_pose = del_pose;
    else del_pose = -del_pose;

    //if (Dist_R < MIN_DIST) {
    //    brk_cmd = 1;
    //    GPS_waypoint_Lat_now = GPS_waypoint_Lat[idx_way];
    //    GPS_waypoint_Lon_now = GPS_waypoint_Lon[idx_way];
    //    idx_way++;
    //    printf("way point: %d\a\n", idx_way);
    //}
    if (deliveryB_flag)
    {
        if (idx_way >= (DeliveryIDX[DelB][StartIDX] - 1) && idx_way <= DeliveryIDX[DelB][FinalIDX])
        {
            printf("check IDX: %d\t%f\t%f\n", idx_way, del_offsetM, del_pose);

            LAT_OFFSET = (del_offsetM * sin(del_pose * DEG2RAD)) / LAT2METER;
            LON_OFFSET = -(del_offsetM * cos(del_pose * DEG2RAD)) / LON2METER;
            printf("check flag: %d\n", deliveryB_flag);

            Waypoint_lat_cal = GPS_waypoint_Lat[idx_way];
            Waypoint_lon_cal = GPS_waypoint_Lon[idx_way];

            GPS_waypoint_Lat_now = Waypoint_lat_cal + LAT_OFFSET;
            GPS_waypoint_Lon_now = Waypoint_lon_cal + LON_OFFSET;

            if (Dist_R < global_minD) {//MIN_DIST) {
                idx_way++;
            }

            printf("delB point waypoint: %d\n", idx_way);
        }
        else
        {
            deliveryB_flag = 0;
        }
    }
    else
    {
        if (Dist_R < global_minD) {//MIN_DIST) {
            brk_cmd = 1;
            GPS_waypoint_Lat_now = GPS_waypoint_Lat[idx_way];
            GPS_waypoint_Lon_now = GPS_waypoint_Lon[idx_way];
            idx_way++;
            printf("way point: %d\a\n", idx_way);
        }
    }
    Save_MIDwaypoint();
}

void SaveData_Guidance()
{
    char Filename[20];// = "_GuidanceData.csv";
    char* hat = "_GuidanceData.csv";
    strcpy(Filename, hat);

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
        fprintf(dataFile, "%d, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f\n",
            (int)buf_Index[r], (double)buf_delta[r], (double)buf_lambda[r], (double)buf_Dist_R[r], (double)buf_gamma[r], (double)buf_delta_f[r],
            (double)buf_waylatnow[r], (double)buf_waylonnow[r], (double)buf_velRead[r], (double)buf_steerRead[r]);
    }

    printf("Guidance Data Successful.\n");

    fclose(dataFile);
}

#endif // !GLOBAL_GUIDANCE