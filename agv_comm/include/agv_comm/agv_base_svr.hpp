#ifndef AGV_BASE_SVR_H
#define AGV_BASE_SVR_H

#include<agv_comm/agv_tcp_ctrl.hpp>

#define BASE_SVR_BUFF_SIZE 1024*1024




st_tcp_connect_config * base_svr_config_create();
int base_svr_config_delete(st_tcp_connect_config *pt_config);


#endif // AGV_BASE_SVR_H




