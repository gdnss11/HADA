clear all; close all; clc;

LAT2M = 110975.575908909;
LON2M = 88743.5932955675;
addpath('C:\Users\okojo\OneDrive\바탕화~1-DESKTOP-HUF3VDO-129301\REVISION_0922_version_Z2J\REVISION_0911')

deliv_wp = load('RawData_0921_static_left.csv');
del_lat = deliv_wp(:,2);
del_lon = deliv_wp(:,3);

LEN_del = length(del_lat);

std = 2;             %[m]: waypoint간격 바꾸고 싶다면 이 숫자를 바꾸면 됨.
new_i = 1;
real_i = 1;
j= 1;
del_latM = del_lat * LAT2M;
del_lonM = del_lon * LON2M;

d_LAT = del_lat(1);
d_LON = del_lon(1);

for i = 1:LEN_del
    distance = sqrt((del_latM(new_i) - del_latM(i))^2 + (del_lonM(new_i) - del_lonM(i))^2)
    
    if distance > std
        new_i = i;

    end
    
    j = j+1;    % 2부터 돌아감
    %[d_LAT(j), d_LON(j)] = [del_lat(new_i), del_lon(new_i)]
    d_LAT(j) = del_lat(new_i);
    d_LON(j) = del_lon(new_i);


    
    if j >= 2
        if d_LAT(j-1) ~= d_LAT(j) && d_LON(j-1) ~= d_LON(j)
            real_LAT(real_i) = d_LAT(j-1);
            real_LON(real_i) = d_LON(j-1);
            real_i = real_i+1;
        end
    end
    
end

cnt_waypoint = length(real_LAT);
cnt_waypoint = repmat(cnt_waypoint, length(real_LAT), 1);
lat_waypoint = real_LAT';
lon_waypoint = real_LON';
waypoint = [cnt_waypoint lat_waypoint lon_waypoint];
writematrix(waypoint, 'way_point_Chapel_ap.csv')
% data = load('way_point_handong.csv');
% geoplot(data(:,2), data(:,3), 'r*');
% geobasemap('satellite')
geoplot(real_LAT, real_LON, 'r*');%, del_lat, del_lon, 'ro', pic_lat, pic_lon, 'yo');
geobasemap('satellite')