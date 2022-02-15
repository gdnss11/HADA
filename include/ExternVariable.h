extern int    count;
extern double simu_t, delta_f;
extern int    vel_cmd, brk_cmd;

extern int    cport_nr;


extern int    RecvArray_rubber[], STOP_LINE[], idx_sign[], OBS_WP[], DEL_A_WP[], DEL_B_WP[];
extern double RecvArray_Line[];										 
extern int    RecvArray_trafficSign[];
extern int    prev_Traffic_flag;
extern int    buf_signflag[][BUF_SIZE];

extern double global_minD;
extern double global_K;


extern double Lidar_dis[];
extern double Lidar_xCoor[];
extern double Lidar_yCoor[];
extern int    flag_cluster;
extern double delta_f, Dist_R;
extern int    simul_cnt;

extern double    yaw_OFFSET;
extern double    yaw_buffer;
extern int       fixqual;
extern double    lat_filtered;
extern double    lon_filtered;
extern double    eX_[][1];
extern int       idx_max[];
extern double    buf_velRead[], buf_steerRead[];

extern double LAT0, LON0;
extern double latitude, longitude, velocity, direction;
extern double accX, accY, accZ, roll, pitch, yaw;
extern int    buf_Index[], buf_Index2[];
extern int 	  xsensMOD, novatelMOD, lidarMOD, erpMOD,
			  globalMOD, localMOD, parkingMOD, clusterMOD, deliveryMOD, obstacleMOD, trafficsignMOD,
			  camMOD, camobMOD, camlaneMOD, camruMOD;

extern double GPS_waypoint_Lat_now, GPS_waypoint_Lon_now;
extern int    DeliveryIDX[][2];


extern int     Delivery_flagA1;
extern int     Delivery_flagA2;
extern int     Delivery_flagA3;
extern int     Delivery_flagB1;
extern int     Delivery_flagB2;
extern int     Delivery_flagB3;


extern int	  ERP_flag, Start_point, deliveryB_flag;

extern double   Way_dist, GPS_waypoint_Lat[], GPS_waypoint_Lon[], Min_dist;

extern int  UNPT_LINE, LEFT_LINE;
extern int  idx_park;
extern int  idx_obs[];

extern int    flag_stop_kid;
extern int    vehicle_gear;
extern int    idx_way, stop_idx;
extern int    RS232_OpenComport(int comport_number, int baudrate, const char* mode, int flowctrl);
extern void   Idle_Time();
extern void   wait_time(double _time);
extern void   InitTImeCheck();
extern void   Import_Xsens();
extern void   Import_Novatel();
extern void   CMD_Guidance2erp(int _spdcmd, int _brkcmd);



