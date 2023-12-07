/*********************************************************************
 * AgvTfEntry AgvTf 类定义
 * 管理AGV传感和运动的坐标系变换
 * Copyright: BGI
 * Author: yang.cheng
 *********************************************************************/
#include <string>
#include <mutex>
#include <thread>
#include <time.h>

#include "cartographer/mapping/pose_extrapolator.h"

#include "AgvTf.h"
#include "agv_type.h"
#include "SlamUtil.h"

/**
 * @brief AgvTfEntry构造 描述一个^{parent}_{child}T变换
 * @param parent 父坐标系名称
 * @param child  子坐标系名称
 * @return
 */
AgvTfEntry::AgvTfEntry(string parent, string child, bool isStatic)
    :parent_(parent), child_(child), isStatic_(isStatic)
{
    //todo transform初始化
    Rigid3d r3I;
    tf_buffer_.push_back(r3I);
}

/**
 * @brief AgvTfEntry析构
 * @param 
 * @return
 */
AgvTfEntry::~AgvTfEntry()
{
}


/**
 * @brief 以当前Linux标准时间设置坐标系变换
 * @param transform 设定的变换
 * @return
 */
int AgvTfEntry::Set(Rigid3d transform)
{
    if(isStatic_)
    {
        return -1;
    }
    //获取系统时间
    struct timespec time_now;
    clock_gettime(CLOCK_REALTIME, &time_now);
    //设置变换
    std::lock_guard<std::mutex> lock(mutex_);
    transform_ = transform;
    timestamp_ = time_now.tv_sec * 1e9 + time_now.tv_nsec;
    return 0;
}

/**
 * @brief 设置坐标系变换
 * @param transform 设定的变换
 * @param timestamp 当前变换的时刻ns
 * @return
 */
int AgvTfEntry::Set(Rigid3d transform, long int timestamp)
{
    if(isStatic_)
    {
        return -1;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    transform_ = transform;
    timestamp_ = timestamp;
    return 0;
}
/**
 * @brief 设置坐标系变换
 * @param transform 设定的变换
 * @return
 */
int AgvTfEntry::SetTfBuffer(Rigid3d transform)
{
    Rigid3d transform_oldest;
    Rigid3d transform_new;
    Rigid3d transform_delta;
    double distance;
    int split_num;
    double factor;
    double angle_delta;
    double split_distance = 1 * 0.02;//1m/s * 20ms

    std::lock_guard<std::mutex> lock(mutex_);
    if(tf_buffer_.size() < 1)//空的，直接添加
    {
        tf_buffer_.push_back(transform);
    }
    else if(tf_buffer_.size() >= 1)//不空
    {
        //计算位姿增量
        if(tf_buffer_.size() == 1)
        {
            transform_oldest = tf_buffer_.front();
        }
        else
        {
            transform_oldest = tf_buffer_.back();
        }
        
        transform_delta = transform_oldest.inverse() * transform;
        distance= sqrt(transform_delta.translation().x() * transform_delta.translation().x()
                    + transform_delta.translation().y() * transform_delta.translation().y());
        // angle_delta = transform.rotation().angularDistance(transform_oldest.rotation());
        if(distance < 0.0005)
        {
            return 0;
        }
        if(fabs(split_distance) > 1.0e-6)
        {
            split_num = std::ceil(distance / split_distance);
            

            for(int i = 1; i <= split_num; i++)
            {
                factor = (double)i / (double)split_num;
                Eigen::Vector3d translation_new = factor * transform.translation() + (1 - factor) * transform_oldest.translation();
                Eigen::Quaterniond rotation_new = Eigen::Quaterniond(transform_oldest.rotation()).slerp(factor, Eigen::Quaterniond(transform.rotation()));
                transform_new = Rigid3d(translation_new, rotation_new);
                LOG(INFO) << split_num << "[  ]" << transform_new;
                tf_buffer_.push_back(transform_new);
            }
          


        }
    }
    return 0;
}
/**
 * @brief 设置坐标系变换
 * @param transform 设定的变换
 * @param timestamp 当前变换的时刻ns
 * @return
 */
int AgvTfEntry::Get(Rigid3d &transform)
{
    std::lock_guard<std::mutex> lock(mutex_);
    transform = transform_;
    return 0;
}

/**
 * @brief 设置坐标系变换
 * @param transform 设定的变换
 * @param timestamp 当前变换的时刻ns
 * @return
 */
int AgvTfEntry::Get(Rigid3d &transform, long int &timestamp)
{
    std::lock_guard<std::mutex> lock(mutex_);
    transform = transform_;
    timestamp = timestamp_;
    return 0;
}
/**
 * @brief 设置坐标系变换
 * @param transform 设定的变换
 * @param timestamp 当前变换的时刻ns
 * @return
 */
int AgvTfEntry::TrimTfBuffer()
{
    std::lock_guard<std::mutex> lock(mutex_);
    transform_ = tf_buffer_.front();
    
    if(tf_buffer_.size() > 1)
    {
        tf_buffer_.pop_front();
        // LOG(INFO)<<transform_;
    }

    return 0;
}
//---------------------------------------------------------
//Agv坐标系变换管理类的方法描述
/**
 * @brief AgvTf 构造
 * @param 
 * @return
 */
AgvTf::AgvTf(/* args */):
    tf_world2map_("world", "map", false),
    tf_map2odom_("map", "odom", false),
    tf_odom2base_("odom", "base_link", false),
    tf_base2laser_("base_link", "laser", false)
{
    ptExpl_ = new PoseExtrapolator(::cartographer::common::FromSeconds(0.001), 9.806);

}

/**
 * @brief 初始化坐标变换树 根据配置文件读取传感器安装位姿信息（目前仅有Laser）
 * @param (todo 传入配置文件)
 * @param 
 * @return
 */
int AgvTf::Init()
{
    int s32Ret;

    TAgvPose2D tB2LPose2D;
    tB2LPose2D.x = 0.298327;
    tB2LPose2D.y = 0.00487942;
    tB2LPose2D.phi = 0.00297122;
    Rigid3d B2L = ToRigid3d(tB2LPose2D);
    // TAgvPose3D pose;
    // pose.position = {0.28812, 0.0, 0.0}; //MiniKiva 激光雷达安装相对于轮距中心
    // pose.orientation = {0.0, 0.0, 0.0, 1.0};
    // Rigid3d base2laser3d = Rigid3d({pose.position.x, pose.position.y, pose.position.z},
    //             ToEigen(pose.orientation));
    tf_base2laser_.Set(B2L);

    //初始化外推器
    TLinuxTime tLinuxTime;
    Rigid3d r3OrgPose; 
    LinuxTimeNow(&tLinuxTime);
    ptExpl_->AddPose(FromLinuxTime(tLinuxTime), r3OrgPose); //将初始位姿{SE3(0)}添加至外推器

    return 0;
    // #endif
}

/**
 * @brief AgvTf析构
 * @param 
 * @return
 */
AgvTf::~AgvTf()
{

}

/**
 * @brief 设置坐标变换
 * @param 
 * @return
 */
int AgvTf::SetTransform(const string &source_frame_id, const string &target_frame_id, Rigid3d transform)
{
    if (source_frame_id == "world" && target_frame_id == "map")
    {
        this->tf_world2map_.Set(transform);
        return 0;
    }
    
    else if (source_frame_id == "map" && target_frame_id == "odom")
    {
        this->tf_map2odom_.Set(transform);
        // tf_map2odom_.SetTfBuffer(transform);
        return 0;
    }

    else if (source_frame_id == "odom" && target_frame_id == "base_link")
    {
        this->tf_odom2base_.Set(transform);
        return 0;
    }
    else
    {
        return -1;
    }

}

/**
 * @brief 设置坐标变换(重载)
 * @param 
 * @return
 */
int AgvTf::SetTransform(const string &source_frame_id, const string &target_frame_id, Rigid3d transform, bool is_buffer)
{
    if (source_frame_id == "world" && target_frame_id == "map")
    {
        if(is_buffer)
        {
            this->tf_world2map_.SetTfBuffer(transform);
        }
        else
        {
            this->tf_world2map_.Set(transform);
        }
        return 0;
    }
    
    else if (source_frame_id == "map" && target_frame_id == "odom")
    {
        if(is_buffer)
        {
            this->tf_map2odom_.SetTfBuffer(transform);
        }
        else
        {
            this->tf_map2odom_.Set(transform);
        }
        return 0;
    }

    else if (source_frame_id == "odom" && target_frame_id == "base_link")
    {
        if(is_buffer)
        {
            this->tf_odom2base_.SetTfBuffer(transform);
        }
        else
        {
            this->tf_odom2base_.Set(transform);
        }
        
        return 0;
    }
    else
    {
        return -1;
    }

}

/**
 * @brief 获取坐标变换
 * @param 
 * @return
 */
int AgvTf::GetTransform(const string &source_frame_id, const string &target_frame_id, Rigid3d &transform)
{
    Rigid3d transform_1, transform_2, transform_3;

    if (source_frame_id == "map" && target_frame_id == "odom")
    {
        tf_map2odom_.Get(transform);
        return 0;
    }
    else if (source_frame_id == "odom" && target_frame_id == "base_link")
    {
        tf_odom2base_.Get(transform);
        return 0;
    }
    else if (source_frame_id == "base_link" && target_frame_id == "laser")
    {
        tf_base2laser_.Get(transform);
        return 0;
    }
    else if (source_frame_id == "map" && target_frame_id == "base_link")
    {
        if (0 == tf_map2odom_.Get(transform_1) 
            && 0 == tf_odom2base_.Get(transform_2))
        {
            transform = transform_1 * transform_2;
            return 0;
        }
    }

    //获取Odom->Laser变换 建立局部地图用于安全策略
    else if (source_frame_id == "odom" && target_frame_id == "laser")
    {
        if (0 == tf_odom2base_.Get(transform_1) 
            && 0 == tf_base2laser_.Get(transform_2))
        {
            transform = transform_1 * transform_2;
            return 0;
        }
    }

    //World to map
    else if (source_frame_id == "world" && target_frame_id == "map")
    {
        tf_world2map_.Get(transform);
    }
    else if (source_frame_id == "world" && target_frame_id == "base_link")
    {
        if (0 == tf_world2map_.Get(transform_1) 
            && 0 == tf_map2odom_.Get(transform_2)
            && 0 == tf_odom2base_.Get(transform_3))
        {
            transform = transform_1 * transform_2 * transform_3;
            return 0;
        }
    }


    return -1;

}
/**
 * @name: AddPoseToExtrapolator
 * @des:  在SLAM 前端回调中插入SLAM前端位姿
 * @param {Rigid3d} r3Pose
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvTf::AddPoseToExtrapolator(Rigid3d r3Pose)
{
    std::lock_guard<std::mutex> lock(mutexExpl_);
    TLinuxTime tLinuxTime;
    LinuxTimeNow(&tLinuxTime);
    ptExpl_->AddPose(FromLinuxTime(tLinuxTime), r3Pose); //将初始位姿{SE3(0)}添加至外推器

    return 0;
}

/**
 * @name: ResetExtrapolator
 * @des:  重置外层外推器
 * @param {Rigid3d} r3Pose
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvTf::ResetExtrapolator()
{
    std::lock_guard<std::mutex> lock(mutexExpl_);
    if (ptExpl_ != NULL)
    {
        delete ptExpl_;
        ptExpl_ = NULL;
    }
    ptExpl_ = new PoseExtrapolator(::cartographer::common::FromSeconds(0.001), 9.806);
    Rigid3d r3OrgPose; 
    TLinuxTime tLinuxTime;
    LinuxTimeNow(&tLinuxTime);
    ptExpl_->AddPose(FromLinuxTime(tLinuxTime), r3OrgPose); //将初始位姿{SE3(0)}添加至外推器

    return 0;
}

/**
 * @name: AddOdomToExtrapolator
 * @des:  在SLAM 前端回调中插入里程计数据
 * @param {Rigid3d} r3Pose
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvTf::AddOdomToExtrapolator(TAgvOdomMsg tOdomMsg)
{
    std::lock_guard<std::mutex> lock(mutexExpl_);

    const cartographer::common::Time time = FromSysTime(tOdomMsg.header.stamp);
    ptExpl_->AddOdometryData(cartographer::sensor::OdometryData{time, ToRigid3d(tOdomMsg.pose)});


    return 0;
}

/**
 * @name: GetGlobalPose
 * @des:  根据当前时刻和外推器获取当前全局位姿
 * @param {Rigid3d} &r3Pose
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvTf::GetGlobalPose3D(Rigid3d &r3GlobalPose)
{
    //由外推器获取当前局部位姿
    int s32Ret;
    Rigid3d r3LocalPose;
    {
        std::lock_guard<std::mutex> lock(mutexExpl_);
        TLinuxTime tLinuxTime;
        LinuxTimeNow(&tLinuxTime);
        r3LocalPose = ptExpl_->ExtrapolatePose(FromLinuxTime(tLinuxTime));
    }

    //获取全局到局部变换
    Rigid3d r3Global2Local;
    s32Ret = GetTransform("map", "odom", r3Global2Local);
    if (s32Ret < 0)
    {
        return -1;
    }
    r3GlobalPose = r3Global2Local * r3LocalPose;

    return 0;
}

/**
 * @name: GetGlobalPose2D
 * @des:  
 * @param {TAgvPose2D} &tAgvPose
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvTf::GetGlobalPose2D(TAgvPose2D &tAgvPose)
{   
    int s32Ret;
    Rigid3d r3GlobalPose;
    s32Ret = GetGlobalPose3D(r3GlobalPose);
    if (s32Ret < 0)
    {
        return -1;
    }

    tAgvPose.x = r3GlobalPose.translation().x();
    tAgvPose.y = r3GlobalPose.translation().y();
    tAgvPose.phi = GetYaw(r3GlobalPose.rotation());
    return 0;
}


/**
 * @name: ToLinuxTime
 * @des:  Slam时间转Linux时间
 * @param {Time} time
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
TLinuxTime ToLinuxTime(::cartographer::common::Time time)
{
    int64_t uts_timestamp = ::cartographer::common::ToUniversal(time);
    int64_t ns_since_unix_epoch =
        (uts_timestamp -
        ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds *
            10000000ll) *
        100ll;
        
    TLinuxTime tLinuxTime;
    tLinuxTime.tv_sec = ns_since_unix_epoch / 1000000000;
    tLinuxTime.tv_nsec = ns_since_unix_epoch % 1000000000;
    return tLinuxTime;
}

/**
 * @name: FromLinuxTime
 * @des: Linux时间转SLAM时间
 * @param {TLinuxTime} time
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
::cartographer::common::Time FromLinuxTime(TLinuxTime time)
{
    // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
    // exactly 719162 days before the Unix epoch.
    return ::cartographer::common::FromUniversal(
        (time.tv_sec +
        ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds) *
            10000000ll +
        (time.tv_nsec + 50) / 100);  // + 50 to get the rounding correct.
}

/**
 * @name: 
 * @des: 
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int LinuxTimeNow(TLinuxTime *ptLinuxTime)
{
    if (ptLinuxTime != NULL)
    {
        clock_gettime(CLOCK_REALTIME, ptLinuxTime);
        return 0;
    }
    
    return -1;
}

