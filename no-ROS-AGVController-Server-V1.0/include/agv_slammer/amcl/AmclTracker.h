/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-08-04 10:46:41
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-08-09 15:16:42
 * @FilePath: /agv_controller/include/agv_slammer/amcl/AmclTracker.h
 * @Description: AMCL辅助位姿跟踪器 精确定位处理
 */
#ifndef AMCLTRACKER_H
#define AMCLTRACKER_H

#include <vector>
#include <mutex>

#include "cartographer/transform/transform.h"
#include "amcl/map/map.h"
#include "amcl/pf/pf.h"
#include "amcl/sensors/amcl_odom.h"
#include "amcl/sensors/amcl_laser.h"

#include "AgvTf.h"
#include "agv_type.h"
#include "agv_map_ctrl.h"

using namespace std;
using namespace amcl;
using ::cartographer::transform::Rigid3d;
using ::cartographer::transform::Rigid2d;


/**
 * @name: amcl_hyp_t
 * @des:  位姿估计相关结构
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
//位姿假设
typedef struct
{
  double weight; //权重
  pf_vector_t pf_pose_mean; //值
  pf_matrix_t pf_pose_cov;  //方差
}amcl_hyp_t;

/**
 * @name: AmclTracker
 * @des:  粒子滤波跟踪器类
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
class AmclTracker
{
private:
    /* data */
public:
    AmclTracker(AgvTf *poAgvTf);
    ~AmclTracker();

public:
    AgvTf *poAgvTf_; //全局坐标变换
    //共享锁
    mutex mutex_;
    
    //地图数据
    map_t* map_;
    char* mapdata;
    int sx, sy;
    double resolution;

    //粒子模拟
    pf_t *pf_;
    double pf_err_, pf_z_;
    bool pf_init_;
    pf_vector_t pf_odom_pose_;
    double d_thresh_, a_thresh_;
    int resample_interval_;
    int resample_count_;
    double laser_min_range_;
    double laser_max_range_;

    int max_beams_, min_particles_, max_particles_;
    double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
    double alpha_slow_, alpha_fast_;
    double z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_;
    bool do_beamskip_;
    double beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_;
    double laser_likelihood_max_dist_;
    
    //传感器
    AMCLOdom* odom_;
    double conv_init_x_;
    double conv_init_y_;
    double conv_init_phi_;

    AMCLLaser* laser_;

//内部工具函数
public:
    map_t* ConvertMap(const TMapInfo *tMapInfo);
    static pf_vector_t UniformPoseGenerator(void* arg);
    static std::vector<std::pair<int,int> > free_space_indices;

//流程调用函数
public:
    int Init(const TMapInfo *ptMapInfo);
    int Reset(const Rigid3d &r3GlobalOrg, const Rigid3d &r3LocalOrg);
    int LaserScanProcess(const Rigid3d &r3LocalCarto,
        const TAgvLaserScanMsg *ptLaserScan,
        Rigid3d &r3GlobalAmcl);

    int reset_conut;

};

#endif // AMCLTRACKER_H