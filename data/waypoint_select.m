clear all; close all; clc;
%===============================================%
% 코드 사용법
% 1. load에 원하는 waypoint .csv파일 load 
% 2. 원하는 시작점 & 끝점 2군데 click (ginput함수)
% 3. final_p에 시작 + 끝지점 idx 저장
%===============================================%
addpath('C:\Users\okojo\OneDrive\바탕화~1-DESKTOP-HUF3VDO-129301\REVISION_0922_version_Z2J\REVISION_0911')

DATA = load('way_point_handong_N2G.csv');


LAT2M = 110975.575908909;
LON2M = 88743.5932955675;

lat = DATA(:,2);
lon = DATA(:,3);
len = length(DATA(:,1));

figure,
geoplot(lat, lon, 'r*');
hold on;
geobasemap('satellite');
geotickformat('dd');

[lat_c, lon_c] = ginput(2);
len_c = length(lat_c);

MIN_d = 1000

for j = 1:len_c
    for i = 1:len
        Cal_d = sqrt((lat(i)*LAT2M - lat_c(j)*LAT2M)^2 + (lon(i)*LON2M - lon_c(j)*LON2M)^2);
        if MIN_d > Cal_d
            MIN_d = Cal_d;
            final_p(j) = i;
            
        end
    end
    MIN_d = 1000;
    final_p
end

geoplot(lat(final_p(1):final_p(2)), lon(final_p(1):final_p(2)), 'yo')
geobasemap('satellite');
geotickformat('dd');