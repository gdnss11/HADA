% PARKING DATA PLOTTING
clear all; close all; clc; 

addpath("C:\HADA\Mission Algorithm\Parking\Real Implementaion\REVISION_0922_GoodAfternoon\REVISION_0911");
data = load('_ParkingData.csv');

LAT2M = 110975.575908909;
LON2M = 88743.5932955675

Park_WP_X = nonzeros(data(:,1));
Park_WP_Y = nonzeros(data(:,2));
Park_WP_Att = nonzeros(data(:,3));

Carpath_X = nonzeros(data(:,8));
Carpath_Y = nonzeros(data(:,9));

Delta_f = nonzeros(data(:,10));
Vel_cmd = nonzeros(data(:,11));

figure,
subplot(2,1,1);
plot(Delta_f,'bo'); grid on;
title("DELTA_F");
subplot(2,1,2);
plot(Vel_cmd,'ro'); grid on;
title("VELOCITY COMMAND");

figure,
%geoplot(ParkWP_Y/LAT2M, ParkWP_X/LON2M,'yo'); hold on;
geoplot(Carpath_Y, Carpath_X, 'b*'); 
geobasemap('satellite');

% (double)buf_X_ParkingWP[r], (double)buf_Y_ParkingWP[r], (double)buf_ATT_ParkingWP[r],
%          (double)buf_parking_psi[r], (double)buf_parking_vel_cmd[r], (double)buf_parking_w_cmd[r], (double)buf_parking_delta_des[r],
%          (double)buf_parking_Xcarpath[r], (double)buf_parking_Ycarpath[r], (double)buf_parking_delta_f[r], (double)buf_parking_vel_cmd[r], (double)buf_Star_XY[r], (double)buf_parkDist[r]);
%          
         
       