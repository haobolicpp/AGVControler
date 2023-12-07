
#include <stdio.h>
#include <pthread.h>
#include <mqueue.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <net/if.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <unistd.h>

#include "agv_type.h"
#include "agv_cmd.h"
#include "agv_config.h"
#include "json.hpp"

#include "AgvCtrl.h"


using json = nlohmann::json;

int read_agv_config_from_json(AgvCtrl *pt_agv_ctrl)
{
    FILE *pf = nullptr;
    char str_file[512] = {0};
    int ifile_len = 0;
    uint8_t *pfile_buffer = nullptr;

    memset(pt_agv_ctrl->t_agv_base_config.chMapName, 0, sizeof(pt_agv_ctrl->t_agv_base_config.chMapName));
    memset(pt_agv_ctrl->t_agv_base_config.chMapMD5, 0, sizeof(pt_agv_ctrl->t_agv_base_config.chMapMD5));

    sprintf(str_file, "/home/%s/Downloads/base_info.json", getlogin());
    pf = fopen(str_file, "r");
    if (pf == nullptr)
    {
        printf("Error: Agv Base Info Config Invalied!\n");
        return -1;
    }
    if (-1 == fseek(pf, 0, SEEK_END))
    {
        return -1;
    }
    if (-1 == (ifile_len=ftell(pf)))
    {
        return -1;
    }
    if (-1 == fseek(pf, 0, SEEK_SET))
    {
        return -1;
    }
    pfile_buffer = (uint8_t*)malloc(ifile_len+1);
    if (pfile_buffer == nullptr)
    {
        return -1;
    }
    memset(pfile_buffer, 0, ifile_len);
    if (1 != fread(pfile_buffer, ifile_len, 1, pf))
    {
        return -1;
    }
    fclose(pf);
    pfile_buffer[ifile_len] = '\0';

    //解析json
    if (!json::accept(pfile_buffer))
    {
        return -1;
    }
    auto itValue = json::parse(pfile_buffer);
    strncpy(pt_agv_ctrl->t_agv_base_config.chMapName, itValue.value("map_name", "").c_str(), sizeof(pt_agv_ctrl->t_agv_base_config.chMapName));
    strncpy(pt_agv_ctrl->t_agv_base_config.chMapMD5, itValue.value("map_md5", "").c_str(), sizeof(pt_agv_ctrl->t_agv_base_config.chMapMD5));

    free(pfile_buffer);

    return 0;
}

int write_agv_config_to_json(AgvCtrl *pt_agv_ctrl)
{
    json jagv_base_config;
    std::string strFileContent;
    FILE *pf = nullptr;
    char str_dir[512] = {0};
    char str_file[512] = {0};
    int ifile_len = 0;

    jagv_base_config["map_name"] = pt_agv_ctrl->t_agv_base_config.chMapName;
    jagv_base_config["map_md5"] = pt_agv_ctrl->t_agv_base_config.chMapMD5;
    strFileContent = jagv_base_config.dump();

    //创建目录
    sprintf(str_dir, "/home/%s/Downloads/", getlogin());
    if (-1 == create_dir_recursive(str_dir))
    {
        return -1;
    }
    
    //写文件
    sprintf(str_file, "%s/base_info.json", str_dir);
    pf = fopen(str_file, "w+");
    if (pf == nullptr)
    {
        printf("Error: Agv Base Info Config Open Error: %d",errno);
        return -1;
    }
    ifile_len = strFileContent.length();
    if (1 != fwrite(strFileContent.c_str(), ifile_len, 1, pf))
    {
        return -1;
    }
    fflush(pf);
    fclose(pf);
    return 0;
}

int create_dir_recursive(const char *pstr_dir)
{
    char DirName[512] = {0};
    strcpy(DirName, pstr_dir);
    int i, len = strlen(DirName);
    for (i = 1; i < len; i++)
    {
        if (DirName[i] == '/' || i == len - 1)
        {
            if (DirName[i - 1] == '~')
            {
                printf("can't create dir like '~'");
                return -1;
            }
            if (DirName[i] == '/')
            {
                DirName[i] = 0;
            }

            if (0 != access(DirName, F_OK)) //文件不存在
            {
                if (mkdir(DirName, 0777) == -1) //创建目录
                {
                    printf("mkdir error:%d\n", errno);
                    return -1;
                }
            }
            DirName[i] = '/';
        }
    }

    return 0;
}

