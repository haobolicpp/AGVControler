/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-10-12 15:12:24
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-10-12 16:30:31
 * @FilePath: /agv_controller/src/agv_slammer/AgvSlammer.cc
 * @Description: SLAM算法接口实现
 */

#include <pthread.h>
#include <mqueue.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "tlog.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "cartographer/io/color.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/transform/rigid_transform.h"

#include "AgvSlammer.h"
#include "SlamUtil.h"
#include "AgvCtrl.h"
#include "agv_type.h"
#include "agv_cmd.h"

DEFINE_string(configuration_directory, "/home/bgi/agv_controller/config",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename_map, "AgvSlammerMap.lua",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

DEFINE_string(configuration_basename_loc, "AgvSlammerLoc.lua",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

/**
 * @brief AgvSlammer 构造
 * @param 
 * @return 
 */
AgvSlammer::AgvSlammer(AgvCtrl *pt_agv_ctrl)
{
    bHasInited = false;
    bHasStarted = false;
    eState = AgvSlammerState::INIT;
    s32CtrlFreq = 20;
    pt_agv_ctrl_ = pt_agv_ctrl;
    MapOptimizationDone = false;

    pLogSlam = NULL;
    bLogSlam = false;

    s32SafetyLevel_ = 0;
    poAmclTracker_ = new AmclTracker(pt_agv_ctrl->poAgvTf);
    poSafetyMove_ = new SafetyMove(pt_agv_ctrl->poAgvTf);

}

/**
 * @brief AgvSlammer 析构
 * @param 
 * @return 
 */
AgvSlammer::~AgvSlammer()
{
    delete poAmclTracker_;
    delete poSafetyMove_;
}

/**
 * @brief AgvSlammer 模块初始化
 * @param 
 * @return 
 */
int AgvSlammer::Init()
{
    int s32ret;

    if (s32CtrlFreq <= 1)
    {
        printf("AgvSlammer too small control freqency!");
        return -1;
    }

    poSafetyMove_->Init();

    //从LUA文件中读取模块参数信息
    std::tie(node_options_, trajectory_options_) = 
        LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename_loc);
    //创建一个Carto Mapbuilder 对象实例
    auto map_builder_ = cartographer::mapping::
        CreateMapBuilder(node_options_.map_builder_options);
    //MapBuilder类对象实例
    upAgvMapBuilder_ = absl::make_unique<AgvMapBuilder>(node_options_, true,
        std::move(map_builder_), pt_agv_ctrl_, pt_agv_ctrl_->poAgvTf);

    eState = AgvSlammerState::LOCALIZATION;
    
    //加载默认地图-上次使用的地图并开启新的轨迹用于AGV定位
    std::string dir = absl::StrCat(absl::StrCat("/home/", getlogin()), "/Downloads");
    LoadState(absl::StrCat(dir, "/Default.pbstream"), true);
    poAmclTracker_->Init(&tMapInfo);
    StartTrajectory(trajectory_options_);

    // //定义初始化地图模块
    pAgvMapAssembler_ = absl::make_unique<AgvMapAssembler>(pt_agv_ctrl_, pt_agv_ctrl_->poAgvTf);
    if (0 != pAgvMapAssembler_->Init())
    {
        return -1;
    }

    //TODO 集成进系统 标准消息定义
    struct mq_attr t_mq_attr = {0};
	t_mq_attr.mq_flags = 0; //Block Mode
	t_mq_attr.mq_msgsize = sizeof(st_robot_msg); //todo intagerate with total project
	t_mq_attr.mq_maxmsg = 8;

    mq_unlink("/q_agv_slammer");
    mode_t omask;
    omask = umask(0); //返回之前的文件权限默认值，同时修改默认的umask数值为0，这样后面设置文件权限的时候，umask值无影响了（创建文件时，文件权限的计算方式是：umask取反 & 要设定的文件权限）
    this->qSlamSvr = mq_open("/q_agv_slammer", O_RDWR | O_CREAT | O_NONBLOCK , (S_IRWXU | S_IRWXG | S_IRWXO) /* 777 其他用户可修改*/,  &t_mq_attr);
    umask(omask);
    if (this->qSlamSvr < 0)
    {
        printf("q_agv_slammer create failed with error:%s\n",strerror(errno));
    }
    assert(this->qSlamSvr >= 0);

    printf("INFO: Agv Slammer Inited!\n");

    bHasInited = true;
    return 0;

}

/**
 * @brief AgvSlammer 模块运行
 * @param 
 * @return 
 */
int AgvSlammer::Start()
{
    int s32Ret;

    if(!bHasInited || bHasStarted)
    {
        printf("ERROR: Agv Slammer Start failed!\n");
        return -1;
    }
    bHasStarted = true;
    
    pthread_attr_t pthread_attr;
    struct sched_param sched_param;
    pthread_attr_init(&pthread_attr);
    pthread_attr_setinheritsched(&pthread_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&pthread_attr, SCHED_OTHER);
    sched_param.sched_priority = 60;
    pthread_attr_setschedparam(&pthread_attr, &sched_param);
    //开启SLAM模块主线程
    s32Ret = pthread_create(&this->thSlamSvr, &pthread_attr, LoopSlamSvrStatic, (void *)(this));
    assert(s32Ret == 0);
    //开启地图组装器模块
    s32Ret = pAgvMapAssembler_->Start();
    printf("INFO: Agv Slammer Run!\n");
    
    return 0;
}


/**
 * @brief AgvPlanner工作线程入口
 * @param arg 当前类指针 
 * @return void
 */
void *AgvSlammer::LoopSlamSvrStatic(void *arg)
{
    int ret;
    struct sched_param param;
    int policy;
    
    ret = pthread_getschedparam(pthread_self(), &policy, &param);
    tlog(TLOG_INFO, "Slam Thread Default Policy[%d], Prioity[%d], ret[%d]\n", policy, param.sched_priority, ret);
    param.sched_priority = 60;
    ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    pthread_getschedparam(pthread_self(), &policy, &param);
    tlog(TLOG_INFO, "Slam Thread SetWith Policy[%d], Prioity[%d], ret[%d]\n", policy, param.sched_priority, ret);

    AgvSlammer *poAgvSlammer = (AgvSlammer *)arg;
    poAgvSlammer->LoopSlamSvr();

    return NULL;
}

/**
 * @brief AgvSlammer工作线程实体
 * @return 
 */
int AgvSlammer::LoopSlamSvr()
{
    int s32Ret;
    struct timeval t_tmptv;
    fd_set tFdReadSet;
    TRobotMsg tAgvMsg;

    int s32FdSetMax = qSlamSvr + 1;
    int usFromCtrlFreq = floor(1000000.0/s32CtrlFreq);
    t_tmptv.tv_sec = 0;
    t_tmptv.tv_usec = usFromCtrlFreq; 

    while(bHasStarted)
    {
        FD_ZERO(&tFdReadSet);
        FD_SET(qSlamSvr, &tFdReadSet);
        s32Ret = select(s32FdSetMax, &tFdReadSet, NULL, NULL, &t_tmptv);
        if (s32Ret < 0)
        {
            printf("AgvSlamQ Loop Error %s\n", strerror(errno));
            sleep(1);
            continue;
        }
        else if (s32Ret == 0)
        {
            t_tmptv.tv_sec = 0;
            t_tmptv.tv_usec = usFromCtrlFreq; 
            continue;
        }
        //消息处理
        else if (FD_ISSET(qSlamSvr, &tFdReadSet))
        {
            memset(&tAgvMsg, 0, sizeof(TRobotMsg));
            while (true)
            {
                s32Ret = mq_receive(qSlamSvr, (char *)&tAgvMsg,
                            sizeof(TRobotMsg), NULL);
                if (s32Ret < 0)
                {
                    // printf("ERROR: Agv Slammer Get Msg Failed %d\n", errno);
                    break;
                }
                s32Ret = AsyncMsgHandler(&tAgvMsg);
                RobotMsgRelease(&tAgvMsg);
            }
        }
    }

    return s32Ret;
}

/**
 * @brief 解析用于定位和建图功能的传感器配置
 * @param options 之前根据LUA解析得到的用户配置
 * @return expected_topics 传感器话题集合
 */
std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
AgvSlammer::ComputeExpectedSensorIds(const TrajectoryOptions& options) const
{
    using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
    using SensorType = SensorId::SensorType;
    std::set<SensorId> expected_topics;
// Subscribe to all laser scan, multi echo laser scan, and point cloud topics.
    for (const std::string& topic :
        ComputeRepeatedTopicNames(kLaserScanTopic, options.num_laser_scans))
    {
        expected_topics.insert(SensorId{SensorType::RANGE, topic});
    }
    for (const std::string& topic : ComputeRepeatedTopicNames(
        kMultiEchoLaserScanTopic, options.num_multi_echo_laser_scans))
    {
        expected_topics.insert(SensorId{SensorType::RANGE, topic});
    }
    for (const std::string& topic :
        ComputeRepeatedTopicNames(kPointCloud2Topic, options.num_point_clouds))
    {
        expected_topics.insert(SensorId{SensorType::RANGE, topic});
    }
// For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
// required.
    if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
        (node_options_.map_builder_options.use_trajectory_builder_2d() &&
        options.trajectory_builder_options.trajectory_builder_2d_options().use_imu_data())
        )
    {
        expected_topics.insert(SensorId{SensorType::IMU, kImuTopic});
    }
// Odometry is optional.
    if (options.use_odometry)
    {
        expected_topics.insert(SensorId{SensorType::ODOMETRY, kOdometryTopic});
    }
// NavSatFix is optional.
    if (options.use_nav_sat) 
    {
        expected_topics.insert(SensorId{SensorType::FIXED_FRAME_POSE, kNavSatFixTopic});
    }
// Landmark is optional.
    if (options.use_landmarks)
    {
        expected_topics.insert(SensorId{SensorType::LANDMARK, kLandmarkTopic});
    }

    return expected_topics;
}

/**
 * @brief 加载默认地图
 * @param state_filename 默认地图名称（pbstram文件）
 * @return expected_topics 传感器话题集合
 */
void AgvSlammer::LoadState(const std::string& state_filename, const bool load_frozen_state) 
{
    if (upAgvMapBuilder_ == nullptr)
    {
        return;
    }

    absl::MutexLock lock(&mutex_);
    upAgvMapBuilder_->LoadState(state_filename, load_frozen_state);

    //加载栅格地图
    ::cartographer::io::ProtoStreamReader reader(state_filename);
    ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);
    std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice> submap_slices;
    ::cartographer::mapping::ValueConversionTables conversion_tables;
    ::cartographer::io::DeserializeAndFillSubmapSlices(
      &deserializer, &submap_slices, &conversion_tables);
      std::cout << "submap_slices.size():" << submap_slices.size();

    //转换栅格地图并加载 AMCL辅助定位模块使用栅格地图
    TLinuxTime tLinuxTime;
    TAgvTimeStamp tAgvTimeStamp;
    LinuxTimeNow(&tLinuxTime);
    tAgvTimeStamp.sec = tLinuxTime.tv_sec;
    tAgvTimeStamp.nsec = tLinuxTime.tv_nsec;
    double resolution = 0.05;
    const auto painted_slices = ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);
    // Rotate();painted_slices.surface
    TOccupancyGrid msg = 
        pAgvMapAssembler_->CreateOccupancyGridMsg(painted_slices, resolution, "map", tAgvTimeStamp);
    pt_agv_ctrl_->SetMapInfo(msg);
    printf("INFO: AgvSlammer Load State With Map W[%d]-H[%d]-OrgX[%f]-OrgY[%f]\n",
        msg.width, msg.height, msg.origin.position.x, msg.origin.position.y);
    
    //初始化 AMCL辅助定位器
    pt_agv_ctrl_->tMapGrid_ = msg;
    pt_agv_ctrl_->GetMapInfo(tMapInfo);
    // upAgvMapBuilder_->poAmclTracker_->Init(&tMapInfo);
}

/**
 * @brief 轨迹参数合法性检验
 * @param options 轨迹参数
 * @return true 合法 false 非法
 */
bool AgvSlammer::ValidateTrajectoryOptions(const TrajectoryOptions& options)
{
    if (node_options_.map_builder_options.use_trajectory_builder_2d())
    {
        return options.trajectory_builder_options.has_trajectory_builder_2d_options();
    }

    if (node_options_.map_builder_options.use_trajectory_builder_3d())
    {
        return options.trajectory_builder_options.has_trajectory_builder_3d_options();
    }

    return false;
}


/**
 * @brief 开启一个新的AGV运动轨迹
 * @param options 轨迹参数
 * @return true 合法 false 非法
 */
int AgvSlammer::AddTrajectory(const TrajectoryOptions& options)
{
    using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
    const std::set<SensorId> expected_sensor_ids = ComputeExpectedSensorIds(options);
    const int trajectory_id = upAgvMapBuilder_->AddTrajectory(expected_sensor_ids, options);

    //增加采样器 可以适当降低传感器数据的处理频率
    AddSensorSamplers(trajectory_id, options);

    return trajectory_id;
}

/**
 * @brief 添加一个外推器用于推断AGV实时位姿
 * @param trajectory_id 针对于该轨迹id的外推器
 * @param options 轨迹参数
 * @return
 */
void AgvSlammer::AddExtrapolator(const int trajectory_id, const TrajectoryOptions& options) 
{
    constexpr double kExtrapolationEstimationTimeSec = 0.001;  // 1 ms
    CHECK(extrapolators_.count(trajectory_id) == 0);
    const double gravity_time_constant =
    node_options_.map_builder_options.use_trajectory_builder_3d()
        ? options.trajectory_builder_options.trajectory_builder_3d_options()
            .imu_gravity_time_constant()
        : options.trajectory_builder_options.trajectory_builder_2d_options()
            .imu_gravity_time_constant();
    extrapolators_.emplace(
    std::piecewise_construct, std::forward_as_tuple(trajectory_id),
    std::forward_as_tuple(
        ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
        gravity_time_constant));
}

/**
 * @brief 按照配置为相关的传感器添加采样器
 * @param trajectory_id 针对于该轨迹id的外推器
 * @param options 轨迹参数
 * @return
 */
void AgvSlammer::AddSensorSamplers(const int trajectory_id, const TrajectoryOptions& options)
{
    CHECK(sensor_samplers_.count(trajectory_id) == 0);
    sensor_samplers_.emplace(
        std::piecewise_construct, std::forward_as_tuple(trajectory_id),
        std::forward_as_tuple(
        options.rangefinder_sampling_ratio,
        options.odometry_sampling_ratio,
        options.fixed_frame_pose_sampling_ratio,
        options.imu_sampling_ratio,
        options.landmarks_sampling_ratio)
        );//emplace;
}

/**
 * @brief 开启建图或定位过程
 * @param trajectory_id 针对于该轨迹id的外推器
 * @param options 轨迹参数
 * @return
 */
void AgvSlammer::StartTrajectory(const TrajectoryOptions& options)
{
  absl::MutexLock lock(&mutex_);
  CHECK(ValidateTrajectoryOptions(options));
  AddTrajectory(options);
}

/**
 * @name: AsyncMsgPost
 * @des:  模块功能请求
 * @param {TRobotMsg} *ptAgvMsg
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvSlammer::AsyncMsgPost(TRobotMsg *ptAgvMsg)
{
    int ret;

    // 调试消息队列的消息数量
    // struct mq_attr slam_mq_attr = {0};
    // mq_getattr(qSlamSvr, &slam_mq_attr);
    // printf("Warn: Slam Q Msg [%ld]\n", slam_mq_attr.mq_curmsgs);

    ret = mq_send(qSlamSvr, (char *)ptAgvMsg, sizeof(TRobotMsg), 0);
    if (ret < 0)
    {
        RobotMsgRelease(ptAgvMsg);
        printf("ERROR: Agv Slammer Async Msg Post: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}
/**
 * @brief slam异步消息处理
 * @param 
 * @return 0 成功 -1失败
 */
int AgvSlammer::AsyncMsgHandler(TRobotMsg *ptAgvMsg)
{
    int s32Ret = 0;
    uint32_t u32_length = 0;

    uint8_t *pMsgBody;
    TAgvLaserScanMsg laserMsg;
    TAgvOdomMsg odomMsg;
    TAgvImuMsg imuMsg;

    std::string filename = "";
    std::string dir;

    dir = absl::StrCat(absl::StrCat("/home/", getlogin()), "/Downloads");

    int trajectory_id = upAgvMapBuilder_->map_builder_->num_trajectory_builders() - 1;
    //激光数据处理
    if(ptAgvMsg->t_msg_header.u16_class == AGV_SLAM_C_SENSOR && 
            ptAgvMsg->t_msg_header.u16_type == AGV_SLAM_T_SENSOR_LASER)
    {

        static int laserSample = 0;
        if (laserSample++ % 2 == 0)
        {
            return 0;
        }

        absl::MutexLock lock(&mutex_);
        if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
            return 0;
        }
        //反序列化
        assert(ptAgvMsg->t_msg_header.s32_len == sizeof(TAgvLaserScanMsg));
        pMsgBody = RobotMsgGetBody(ptAgvMsg);

        memcpy(&laserMsg, pMsgBody, sizeof(TAgvLaserScanMsg));
        memcpy(&tCurLaserScan, pMsgBody, sizeof(TAgvLaserScanMsg));
        
        upAgvMapBuilder_->sensor_bridges_.at(trajectory_id).
            get()->HandleLaserScanMessage(kLaserScanTopic, &laserMsg);

    }
    //odom数据处理
    else if(ptAgvMsg->t_msg_header.u16_class == AGV_SLAM_C_SENSOR && 
            ptAgvMsg->t_msg_header.u16_type == AGV_SLAM_T_SENSOR_ODOM &&
            trajectory_options_.use_odometry)
    {

        absl::MutexLock lock(&mutex_);
        if (!sensor_samplers_.at(trajectory_id).odometry_sampler.Pulse()) {
            return 0;
        }

        pthread_t th_id = pthread_self();
        pMsgBody = RobotMsgGetBody(ptAgvMsg);
        memcpy(&odomMsg, pMsgBody, sizeof(TAgvOdomMsg));
        upAgvMapBuilder_->sensor_bridges_.at(trajectory_id).get()->HandleOdometryMessage(kOdometryTopic, &odomMsg);
        
    }
    //imu数据处理
    else if(ptAgvMsg->t_msg_header.u16_class == AGV_SLAM_C_SENSOR && 
            ptAgvMsg->t_msg_header.u16_type == AGV_SLAM_T_SENSOR_IMU &&
            trajectory_options_.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())
    {
        //TODO IMU数据融合暂不处理----------------------------------------------
        // absl::MutexLock lock(&mutex_);
        // if (!sensor_samplers_.at(trajectory_id).imu_sampler.Pulse()) {
        //     return 0;
        // }
        // imuMsgDeserializer(ptAgvMsg->t_msg_body.lbody, &u32_length, &imuMsg);
        // upAgvMapBuilder_->sensor_bridges_.at(trajectory_id).get()->HandleImuMessage(kImuTopic, &imuMsg);
        
    }
    //定位、建图模式切换
    else if(ptAgvMsg->t_msg_header.u16_class == AGV_CMD_C_CTRL && 
            ptAgvMsg->t_msg_header.u16_type == AGV_CMD_T_LOCALIZATION_RUN)//建图->定位
    {
        if(eState == AgvSlammerState::SLAM)
        {
            {
                absl::MutexLock locker(&mutex_);
                //建图切换定位，先保存地图
                upAgvMapBuilder_->FinishTrajectory(trajectory_id);
                upAgvMapBuilder_->RunFinalOptimization();
                //保存成pbstream
                filename = absl::StrCat(dir, "/Default.pbstream");
                upAgvMapBuilder_->map_builder_->SerializeStateToFile(false, filename);
                //保存yaml、pgm
                upAgvMapBuilder_->save(filename, absl::StrCat(dir, "/Default"), 0.05);
            }
            //建图结束，地图优化完成
            MapOptimizationDone = true;
            //开始定位
            iLocalization();
        }
        else
        {
            //TODO 切换地图实现
            printf("Warn: Agv Slammer Already In Localization Mode\n");
            return -1;
        }
    }
    else if(ptAgvMsg->t_msg_header.u16_class == AGV_CMD_C_CTRL && 
            ptAgvMsg->t_msg_header.u16_type == AGV_CMD_T_SLAM_RUN)//定位->建图
    {
        if(eState == AgvSlammerState::LOCALIZATION)
        {
            {
                absl::MutexLock locker(&mutex_);
                //建图切换定位
                upAgvMapBuilder_->FinishTrajectory(trajectory_id);
            }
            //定位结束，地图不需要优化
            MapOptimizationDone = false;
            //开始建图
            iMapping();
        }
        else
        {
            printf("Warn: Agv Slammer Already In Mapping Mode\n");
            return -1;
        }
    }

    //开启数据记录
    else if(ptAgvMsg->t_msg_header.u16_class == AGV_LOG_C_CTRL && 
            ptAgvMsg->t_msg_header.u16_type == AGV_LOG_T_START)
    {
        char dirLogSlam[128];
        char timestr[256];
        time_t t = time(0);
        strftime(timestr, 256, "%Y%m%d%H%M%S", localtime(&t));
        sprintf(dirLogSlam, "/home/%s/Downloads/slam_time_log_%s.txt", getlogin(), timestr);
        pLogSlam = fopen(dirLogSlam, "w+");
        assert(pLogSlam != NULL);
        bLogSlam = true;
        
        return 0;
    }

    //关闭数据记录
    else if(ptAgvMsg->t_msg_header.u16_class == AGV_LOG_C_CTRL && 
            ptAgvMsg->t_msg_header.u16_type == AGV_LOG_T_STOP)
    {
        if (bLogSlam)
        {
            bLogSlam = false;
            fclose(pLogSlam);
            pLogSlam = NULL;
        }

        return 0;
    }

    return s32Ret;
}
/**
 * @brief slam定位
 * @param 
 * @return 0 成功 -1失败
 */
int AgvSlammer::iLocalization()
{
    int s32ret = 0;

    //从LUA文件中读取模块参数信息
    std::tie(node_options_, trajectory_options_) = 
        LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename_loc);
    //创建一个Carto Mapbuilder 对象实例
    auto map_builder_ = cartographer::mapping::
        CreateMapBuilder(node_options_.map_builder_options);

{      
        absl::MutexLock locker(&mutex_);
        //清空upAgvMapBuilder_
        upAgvMapBuilder_.reset();
        // delete upAgvMapBuilder_;
        //新建MapBuilder类对象实例
        upAgvMapBuilder_ = absl::make_unique<AgvMapBuilder>(node_options_, true,
            std::move(map_builder_), pt_agv_ctrl_, pt_agv_ctrl_->poAgvTf);
}
    sensor_samplers_.clear();
    extrapolators_.clear();
    // upAgvMapBuilder_ = new AgvMapBuilder(node_options_, 
    //     std::move(map_builder_), g_pAgvTf_);

    //纯定位模式
    eState = AgvSlammerState::LOCALIZATION;
    pt_agv_ctrl_->poAgvTf->ResetExtrapolator();
    //加载默认地图-上次使用的地图并开启新的轨迹用于AGV定位
    std::string dir = absl::StrCat(absl::StrCat("/home/", getlogin()), "/Downloads");
    LoadState(absl::StrCat(dir, "/Default.pbstream"), true);
    poAmclTracker_->Init(&tMapInfo);
    StartTrajectory(trajectory_options_);
    upAgvMapBuilder_->bGetLocalToGloable = true;

    return s32ret;
}

/**
 * @brief slam建图
 * @param 
 * @return 0 成功 -1失败
 */
int AgvSlammer::iMapping()
{
    int s32ret = 0;

    //从LUA文件中读取模块参数信息
    std::tie(node_options_, trajectory_options_) = 
        LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename_map);
    //创建一个Carto Mapbuilder 对象实例
    auto map_builder_ = cartographer::mapping::
        CreateMapBuilder(node_options_.map_builder_options);
    
    {
        absl::MutexLock locker(&mutex_);
        //清空upAgvMapBuilder_
        upAgvMapBuilder_.reset();
        //新建MapBuilder类对象实例
        upAgvMapBuilder_ = absl::make_unique<AgvMapBuilder>(node_options_, false,
            std::move(map_builder_), pt_agv_ctrl_, pt_agv_ctrl_->poAgvTf);
    }
    sensor_samplers_.clear();
    extrapolators_.clear();

    //建图状态
    eState = AgvSlammerState::SLAM;
    pt_agv_ctrl_->poAgvTf->ResetExtrapolator();

    StartTrajectory(trajectory_options_);
    upAgvMapBuilder_->bGetLocalToGloable = true;
    
    return s32ret;
}
/**
 * @brief 上报地图优化进度
 * @param 
 * @return 地图优化进度
 */
double AgvSlammer::ReportProgress()
{
    absl::MutexLock locker(&mutex_);
    //return upAgvMapBuilder_->map_builder_->pose_graph()->ReportProgress();
    return 1;
}