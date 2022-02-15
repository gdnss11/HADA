#pragma once

#ifndef EKF
#define EKF

#include "Filter.h"

void   NavigationFilter()
{
    ACC_BDY[0][0] = accX;
    ACC_BDY[1][0] = accY;
    ACC_BDY[2][0] = accZ;

    DCM_change(MAT_DCM, roll, pitch, yaw);// mimu[cnt].IMU_roll, mimu[cnt].IMU_pitch, mimu[cnt].IMU_yaw);
    MAT_MULTI(MAT_DCM, ACC_BDY, ACC_NED);

    FN = ACC_NED[0][0];
    FE = ACC_NED[1][0];

    if ((36.0 < latitude) && (latitude < 39.0) && (126.0 < longitude) && (longitude < 130.0) || (latitude != POS_GPS[0][0] && longitude != POS_GPS[0][1]))
    {
        //======SYSTEM UPDATE=======
        System_update();

        //====MEASUREMENT UPDATE====
        POS_GPS[0][0] = latitude;
        POS_GPS[1][0] = longitude;

        MAT_MULTI(MAT_H, pX_, pY);
        MAT_MINUS(POS_GPS, pY, RESIDUAL);
        MAT_MULTI(pP, MAT_H_T, pP_HT);
        MAT_MULTI(MAT_H, pP, H_pP);
        MAT_MULTI(H_pP, MAT_H_T, H_pP_HT);
        MAT_PLUS(H_pP_HT, MAT_R, H_pP_HT_R);
        INVERSE_2X2(H_pP_HT_R, INV_H_pP_HT_R);

        MAT_MULTI(pP_HT, INV_H_pP_HT_R, MAT_K);

        MAT_MULTI(MAT_K, MAT_H, K_H);
        MAT_MINUS(EYE, K_H, EYE_K_H);
        MAT_MULTI(EYE_K_H, pP, eP);

        MAT_MULTI(MAT_K, RESIDUAL, K_RESIDUAL);
        MAT_PLUS(pX_, K_RESIDUAL, eX_);

    }
    else                                     // for real situation
    //else if (42 <= idx_way && idx_way <= 69)   // for 0922 morning experiment
    {
        System_update();

        MAT_MULTI(EYE, pX_, eX_);
        MAT_MULTI(EYE, pP, eP);
    }

    lat_filtered = eX_[0][0];
    lon_filtered = eX_[2][0];

    buf_Filtered_Lat[count] = lat_filtered;// eX[0][0];
    buf_Filtered_Lon[count] = lon_filtered;// eX[2][0];

    buf_RES_LAT[count] = RESIDUAL[0][0] * DEG2RAD * R0;
    buf_RES_LON[count] = RESIDUAL[1][0] * DEG2RAD * R0;
}

void variable(void)
{
    sigPOS_LAT = R_COV / R0 * RAD2DEG;
    sigPOS_LON = R_COV / R0 * RAD2DEG / cos(LAT0 * DEG2RAD);
    MAT_R[0][0] = SQUA(sigPOS_LAT);
    MAT_R[1][1] = SQUA(sigPOS_LON);
}

void INVERSE_2X2(double _MAT[2][2], double _MAT_INV[2][2])
{
    double DET;

    DET = _MAT[0][0] * _MAT[1][1] - _MAT[1][0] * _MAT[0][1];
    _MAT_INV[0][0] = _MAT[1][1] / DET;
    _MAT_INV[0][1] = (-1.0) * _MAT[0][1] / DET;
    _MAT_INV[1][0] = (-1.0) * _MAT[1][0] / DET;
    _MAT_INV[1][1] = _MAT[0][0] / DET;
    _MAT = _MAT_INV;
}

void DCM_change(double _MAT[3][3], double _roll, double _pitch, double _yaw)
{
    double roll_d2r = _roll * DEG2RAD;
    double pitch_d2r = _pitch * DEG2RAD;
    double yaw_d2r = _yaw * DEG2RAD;

    _MAT[0][0] = cos(pitch_d2r) * cos(yaw_d2r);
    _MAT[1][0] = cos(pitch_d2r) * sin(yaw_d2r);
    _MAT[2][0] = -1.0 * sin(pitch_d2r);

    _MAT[0][1] = sin(roll_d2r) * sin(pitch_d2r) * cos(yaw_d2r) - cos(roll_d2r) * sin(yaw_d2r);
    _MAT[1][1] = sin(roll_d2r) * sin(pitch_d2r) * sin(yaw_d2r) + cos(roll_d2r) * cos(yaw_d2r);
    _MAT[2][1] = sin(roll_d2r) * cos(pitch_d2r);

    _MAT[0][2] = cos(roll_d2r) * sin(pitch_d2r) * cos(yaw_d2r) + sin(roll_d2r) * sin(yaw_d2r);
    _MAT[1][2] = cos(roll_d2r) * sin(pitch_d2r) * sin(yaw_d2r) - sin(roll_d2r) * cos(yaw_d2r);
    _MAT[2][2] = cos(roll_d2r) * cos(pitch_d2r);
}

void System_update()
{
    //========================SYSTEM UPDATE==============================
    eX_LAT = eX_[0][0];
    eX_VN = eX_[1][0];
    eX_LON = eX_[2][0];
    eX_VE = eX_[3][0];

    MAT_F[0][0] = 1.0;
    MAT_F[0][1] = SAMPLING_TIME / R0 * RAD2DEG;
    MAT_F[0][2] = 0.0;
    MAT_F[0][3] = 0.0;
    MAT_F[1][0] = SAMPLING_TIME * (-2.0 * We * eX_VE * cos(DEG2RAD * eX_LAT) - SQUA(eX_VE) / (R0 * SQUA(cos(DEG2RAD * eX_LAT))));
    MAT_F[1][1] = 1.0;
    MAT_F[1][2] = 0.0;
    MAT_F[1][3] = SAMPLING_TIME * (-2.0 * We * sin(DEG2RAD * eX_LAT) - 2.0 * tan(DEG2RAD * eX_LAT) * eX_VE / R0);
    MAT_F[2][0] = SAMPLING_TIME * tan(DEG2RAD * eX_LAT) * eX_VE / (R0 * cos(DEG2RAD * eX_LAT)) * RAD2DEG;
    MAT_F[2][1] = 0.0;
    MAT_F[2][2] = 1.0;
    MAT_F[2][3] = SAMPLING_TIME / (R0 * cos(DEG2RAD * eX_LAT)) * RAD2DEG;
    MAT_F[3][0] = SAMPLING_TIME * (2.0 * We * eX_VN * cos(DEG2RAD * eX_LAT) + eX_VE * eX_VN / (R0 * (SQUA(cos(DEG2RAD * eX_LAT)))));
    MAT_F[3][1] = SAMPLING_TIME * (2.0 * We * sin(DEG2RAD * eX_LAT) + tan(DEG2RAD * eX_LAT) * eX_VE / R0);
    MAT_F[3][2] = 0.0;
    MAT_F[3][3] = 1.0 + SAMPLING_TIME * tan(DEG2RAD * eX_LAT) * eX_VN / R0;

    //pX_[0][0] = eX_LAT + SAMPLING_TIME * eX_VN / R0;
    //pX_[1][0] = eX_VN  + SAMPLING_TIME * (FN - 2 * We * eX_VE * sin(eX_LAT * DEG2RAD) - tan(eX_LAT * DEG2RAD) * SQUA(eX_VE) / R0);
    //pX_[2][0] = eX_LON + SAMPLING_TIME * 1 / (R0 * cos(eX_LAT * DEG2RAD)) * eX_VE;
    //pX_[3][0] = eX_VE  + SAMPLING_TIME * (FE + 2 * We * eX_VN * sin(eX_LAT * DEG2RAD) + tan(eX_LAT * DEG2RAD) * eX_VE * eX_VN / R0);
    pX_[0][0] = eX_LAT + SAMPLING_TIME * eX_VN / R0 * RAD2DEG;
    pX_[1][0] = velocity * sin((90 - direction) * DEG2RAD);
    pX_[2][0] = eX_LON + SAMPLING_TIME * eX_VE / (R0 * cos(eX_LAT * DEG2RAD)) * RAD2DEG;
    pX_[3][0] = velocity * cos((90 - direction) * DEG2RAD);

    for (int _Out = 0; _Out < 4; _Out++)
    {
        for (int _In = 0; _In < 4; _In++)
        {
            MAT_F_T[_Out][_In] = MAT_F[_In][_Out];
        }
    }
    MAT_MULTI(MAT_F, eP, F_eP);
    MAT_MULTI(F_eP, MAT_F_T, F_eP_FT);
    MAT_PLUS(F_eP_FT, MAT_Q, pP);
}

void SaveData_Filter()
{
    char Filename[20];// = "_FilterData.csv";
    strcpy(Filename, "_FilterData.csv");

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
        fprintf(dataFile, "%d, %.7f, %.7f, %.7f, %.7f\n",
            (int)buf_Index[r], (double)buf_Filtered_Lat[r], (double)buf_Filtered_Lon[r], (double)buf_RES_LAT[r], (double)buf_RES_LON[r]);
    }

    printf("Filtered Data Successful.\n");

    fclose(dataFile);
}


#endif // !EKF