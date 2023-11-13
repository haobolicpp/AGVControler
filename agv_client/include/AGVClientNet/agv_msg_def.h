#ifndef AGV_MSG_DEF_H
#define AGV_MSG_DEF_H
#include <string>
#include <qmetatype.h>

//Server类型
enum class ETCPClientServerType{
    Type_Control,   //控制类
    Type_File       //文件类
};

//包大小缓冲
#define PACK_SIZE 1024
//连接接收环形缓冲区大小，在接收文件数据时，需动态调整
#define RECV_RING_BUFFER_SIZE 1024*16 //16k
#define RECV_RING_BUFFER_MAX_SIZE 1024*1024*16 //16M

//接收的消息是否是应答消息标记
#define TYPE_IS_ACK_FLAG 0x8000

//构造class和type的联合key
#define MAKE_CLASSTYPE_KEY(class, type) (((class)<<16)|(type))

//连接无数据检测定时时间 ms
#define NODATA_OVERTIME 10000
//发送超时时间 ms
#define SEND_OVERTIME 5000

//间隔符
#define DELIMITER 0xFFFFFFFF

//AGVClient编号
#define AGVCLIENT_ID 0x1000

/*
 *****AGV通信消息定义****
*/
//消息类别定义
typedef enum {
    Class_Control = 0, //控制类
    Class_Data = 1, //数据类

    Class_Internal = 100, //内部使用
}EClassDef;

//控制类消息定义
typedef enum {
    Type_Control_Broadcast = 1,     //广播
    Type_Control_RealTimeData = 2,  //实时上报的数据
    Type_Control_Move = 3,          //AGV移动控制
    Type_Control_GetWifi = 4,       //获取Wifi扫描信息
    Type_Control_SetWifi = 5,       //连接某个Wifi
    Type_Control_GoStation = 6,     //下发目标站点
    Type_Control_StopGoStation = 7, //停止AGV，取消目标站点
    Type_Control_LocalPath = 8,     //实时局部路径
    Type_Control_GlobalPath = 9,    //全局路径
    Type_Control_ModifyReportRate = 10,//修改控制器实时数据上报频率
    Type_Control_Teleoperation = 11,// 遥操作

    Type_Control_Max
}ETypeControlDef;

//数据类消息定义
typedef enum {
    Type_Data_StartScanMap = 1,     //开始扫描地图
    Type_Data_StopScanMap = 2,      //停止扫描地图
    Type_Data_ReportGridMap = 3,    //实时网格地图上报
    Type_Data_DownMapBagData = 4,   //下载地图包数据


    Type_Data_Max
}ETypeDataDef;

//内部消息定义
typedef enum {
    Type_Internal_ForceDisconnect = 1, //AGV强行断开连接
    Type_Internal_BroadcastResponse = 2, //广播应答回来之后，告知两个连接进行连接（跨线程消息通知）

    TypeInternal
}ETypeInternalDef;

///消息头定义
#pragma pack(1)
typedef struct TMsgHeader{
    unsigned int uiDelimiter; //间隔符
    unsigned short shSrc;     //源识别ID
    unsigned short shDes;     //目标识别ID
    unsigned int SerialNum; //序列号
    unsigned short Class;
    unsigned short Type;
    unsigned int uiLen;     //纯数据长度
}TMsgHeader;

///控制类数据定义
//AGV实时数据上报
typedef struct TAGVRealTimeData{
    unsigned char chElec; //电量(0-100)
    unsigned char chRAM; //内存占用百分比
    unsigned char chCPU; //CPU占用
    char chReserved;    //保留
    float fCondfidence; //置信度(0~1)
    float fLinearSpeed; //线速度(m/s)
    float fAngularSpeed; //角速度(rad/s)
    float fAGVX; //AGV X坐标(m)
    float fAGVY; //AGV Y坐标(m)
    float fAngle;//车头朝向角(弧度-PI~PI)
    char chAGVRunMode; //AGV运行模式，0：空闲模式；1：导航模式；2：建图模式；
    char chMapMD5[4]; //地图MD5数值
    char chMapName[32]; //地图名称

}TAGVRealTimeData;

//广播应答
typedef struct TBroadcastResData{
      unsigned int uVersion; //控制器程序版本号
      TAGVRealTimeData tRTData; //实时数据
}TBroadcastResData;

//AGV移动控制
typedef struct TAGVMoveControl{
    float fLinearSpeed; //线速度(m/s)
    float fAngularSpeed; //角速度(rad/s)
}TAGVMoveControl;

//目标站点控制
typedef struct TAGVGoToDes{
    int iStationID; //目标站点
    float fX; //世界坐标系，m
    float fY; //世界坐标系，m
    float fAngle; //(-PI~PI)
}TAGVGoToDes ;

//目标站点应答
typedef struct TAGVGoToDesResponse{
    unsigned char chFlag; //0 成功到达，1超时，2繁忙
} TAGVGoToDesResponse;

///数据类消息定义
//开始扫图 -- 无
//停止扫图 -- 无
//上报的实时网格地图数据
typedef struct TReportGripMap{
    float fResolution; //分辨率 m
    int iHight; //高度(像素)
    int iWidth;  //宽度(像素)
    float dOriginXOffset; //地图左下角x偏移，由此可计算出地图(0,0)点位置，单位m
    float dOriginYOffset; //地图左下角y偏移
    //unsigned char chMapData[0]; //地图数据：0空白，100占用，255未知
}TReportGripMap;

//停止扫图上报的网格地图数据+pbstream数据
typedef struct TReportGripAndStreamMap{
    TReportGripMap tmap;
    //unsigned char chMapData[0]; //地图数据：0空白，100占用，255未知
    //4字节描述pbstream长度
    //pbstream数据
}TReportGripAndStreamMap;

//下发地图包请求数据
typedef struct TDownMapReq{
    int iMapSize; //数据大小(字节)
}TAGVMapDataReq;
#pragma pack()

///内部类消息定义
//断开连接 -- 无
//广播应答:
typedef struct TBroadcastResInter{
    TBroadcastResData tData; //广播应答数据
    char chIP[32]; //IP数据
}TBroadcastResInter;

/*
 *****通信层到上层 数据定义****
*/
//回调上传的基础数据
typedef struct TCallBackData{
    int iref; //引用计数，当一个消息有多个接收者时，通过该引用计数来判断是否销毁数据
    TMsgHeader tHeader;
    void *pData; //通信层收到的数据，会malloc出来，使用完成后会在DeleteCallBackData中释放
}TCallBackData;
//删除TCallBackData指针
inline void DeleteCallBackData(TCallBackData *pData)
{
    pData->iref--;
    if (pData->iref <=0 )
    {
        free (pData->pData);
        delete pData;
    }
}

/*
 *****通信层内部定义****
*/
//UI线程到通信线程的异步发送队列数据定义
typedef struct TSendQAsyncData{
    TMsgHeader tHeader;
    void *pData;
}TSendQAsyncData;

//AGV连接状态
typedef enum{
    AGVConnectStatus_Invalid, //初始
    AGVConnectStatus_Sucess, //连接成功
    AGVConnectStatus_Failure //失败
}EAGVConnectStatus;

//AGV不同连接的连接状态
typedef struct TAGVConnetcStatus{
    EAGVConnectStatus statusControl;    //控制连接的连接状态
    EAGVConnectStatus statusData;       //数据连接的连接状态
    bool bControlDisconnect;            //控制连接是否断开
    bool bDataDisconnect;               //数据连接是否断开
}TAGVConnetcStatus;


#endif // AGV_MSG_DEF_H
