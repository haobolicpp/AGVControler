/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-08-30 10:21:41
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-08-30 11:55:37
 * @FilePath: /agv_controller/src/agv_common/agv_type.cpp
 * @Description: 
 */
#include <stdio.h>
#include <stdint.h>
#include <memory.h>
#include <vector>

#include "agv_type.h"

/**
 * @name: SerializeOccupancyGrid
 * @des:  序列化地图数据用于传输
 * @param {TOccupancyGrid} &tOccupancyGrid
 * @param {uint8_t} *pSerializeOut
 * @param {int} &s32Len
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int SerializeOccupancyGrid(TOccupancyGrid &tOccupancyGrid, uint8_t *pSerializeOut, int &s32LenOut)
{
    //最大长度检查
    int s32Offset = 0;
    int s32DataSize;

    if (OCCUPANCY_GRID_BASE_LEN + tOccupancyGrid.data.size() >= SERIALIZE_LENGTH_MAX)
    {
        return -1;
    }

    //非法数据 地图尺寸与描述不匹配
    s32DataSize = tOccupancyGrid.width * tOccupancyGrid.height;
    if (tOccupancyGrid.data.size() != s32DataSize)
    {
        printf("ERROR: Serialize Map Size Not Match To It's Data!\n");
        return -1;
    }
    //输出目的检查
    if (pSerializeOut == NULL)
    {
        return -1;
    }

    //拷贝基本信息
    s32Offset = 0;
    memcpy(pSerializeOut + s32Offset, &tOccupancyGrid.header, sizeof(TSensorMsgHeader));
    s32Offset = s32Offset + sizeof(TSensorMsgHeader);
    memcpy(pSerializeOut + s32Offset, &tOccupancyGrid.map_load_time, sizeof(TAgvTimeStamp));
    s32Offset = s32Offset + sizeof(TAgvTimeStamp);
    memcpy(pSerializeOut + s32Offset, &tOccupancyGrid.resolution, sizeof(double));
    s32Offset = s32Offset + sizeof(double);
    memcpy(pSerializeOut + s32Offset, &tOccupancyGrid.width, sizeof(uint32_t));
    s32Offset = s32Offset + sizeof(uint32_t);
    memcpy(pSerializeOut + s32Offset, &tOccupancyGrid.height, sizeof(uint32_t));
    s32Offset = s32Offset + sizeof(uint32_t);
    memcpy(pSerializeOut + s32Offset, &tOccupancyGrid.origin, sizeof(TAgvPose3D));
    s32Offset = s32Offset + sizeof(TAgvPose3D);
    //拷贝数据区域
    s32DataSize = sizeof(int8_t) * s32DataSize;
    memcpy(pSerializeOut + s32Offset, tOccupancyGrid.data.data(), s32DataSize);
    s32Offset = s32Offset + s32DataSize;

    //序列化总长
    s32LenOut = s32Offset;

    return 0;
}
/**
 * @name: DeSerilizeOccupancyGrid
 * @des:  反序列化地图数据
 * @param {uint8_t} *pSerializeIn
 * @param {int} &s32LenIn
 * @param {TOccupancyGrid} &tOccupancyGrid
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int DeSerilizeOccupancyGrid(uint8_t *pSerializeIn, int &s32LenIn, TOccupancyGrid &tOccupancyGrid)
{
    int s32Offset = 0;
    int s32DataSize;

    if (pSerializeIn == NULL || s32LenIn <= 0)
    {
        return -1;
    }

    //拷贝基本信息
    s32Offset = 0;
    memcpy(&tOccupancyGrid.header, pSerializeIn + s32Offset, sizeof(TSensorMsgHeader));
    s32Offset = s32Offset + sizeof(TSensorMsgHeader);
    memcpy(&tOccupancyGrid.map_load_time, pSerializeIn + s32Offset, sizeof(TAgvTimeStamp));
    s32Offset = s32Offset + sizeof(TAgvTimeStamp);
    memcpy(&tOccupancyGrid.resolution, pSerializeIn + s32Offset, sizeof(double));
    s32Offset = s32Offset + sizeof(double);
    memcpy(&tOccupancyGrid.width, pSerializeIn + s32Offset, sizeof(uint32_t));
    s32Offset = s32Offset + sizeof(uint32_t);
    memcpy(&tOccupancyGrid.height, pSerializeIn + s32Offset, sizeof(uint32_t));
    s32Offset = s32Offset + sizeof(uint32_t);
    memcpy(&tOccupancyGrid.origin, pSerializeIn + s32Offset, sizeof(TAgvPose3D));
    s32Offset = s32Offset + sizeof(TAgvPose3D);

    //地图数据合法性判断
    s32DataSize = (s32LenIn - OCCUPANCY_GRID_BASE_LEN) / sizeof(int8_t);
    if (s32DataSize != (tOccupancyGrid.width * tOccupancyGrid.height) )
    {
        printf("ERROR: DeSerialize Map Size Not Match To It's Data!\n");
        return -1;
    }
    //拷贝地图数据
    int8_t *pData = (int8_t *)(pSerializeIn + s32Offset);
    tOccupancyGrid.data.resize(s32DataSize);
    for (int i = 0; i < s32DataSize; i++)
    {
        tOccupancyGrid.data.push_back(pData[i]);
    }

    s32Offset = s32Offset + s32DataSize * sizeof(int8_t);


    return 0;
}
/**
 * @name: 
 * @des: 
 * @param {TAgvPointCloud2} &tPointCloud2
 * @param {uint8_t} *pSerializeOut
 * @param {int} &s32LenOut
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int SerializePointCloud2(TAgvPointCloud2 &tPointCloud2, uint8_t *pSerializeOut, int &s32LenOut)
{
    return 0;
}
/**
 * @name: 
 * @des: 
 * @param {uint8_t} *pSerializeIn
 * @param {int} &s32LenIn
 * @param {TAgvPointCloud2} &tPointCloud2
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int DeSerializePointCloud2(uint8_t *pSerializeIn, int &s32LenIn, TAgvPointCloud2 &tPointCloud2)
{
    return 0;
}