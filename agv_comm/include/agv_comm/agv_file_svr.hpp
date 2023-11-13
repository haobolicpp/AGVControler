#ifndef AGV_FILE_SVR_H
#define AGV_FILE_SVR_H

#include<agv_comm/agv_tcp_ctrl.hpp>

#define AGV_FILE_MAX_SIZE 8*1024*1024



st_tcp_connect_config * file_svr_config_create();
int file_svr_config_delete(st_tcp_connect_config *pt_config);



#endif // AGV_FILE_SVR_H