/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-08-04 10:47:47
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-08-14 16:15:56
 * @FilePath: /agv_controller/src/agv_slammer/amcl/AmclTracker.cpp
 * @Description: Amcl精确定位器
 */
#include "amcl/AmclTracker.h"

#include "agv_type.h"
#include "SlamUtil.h"

using namespace amcl;
/**
 * @name: 
 * @des: 
 * @param {* args} *
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
AmclTracker::AmclTracker(AgvTf *poAgvTf)
{
    poAgvTf_ = poAgvTf;

    pf_ = NULL;
    map_ = NULL;
    odom_ = NULL;
    laser_ = NULL;

    reset_conut = 0;

    //初始位姿给定的协方差
    conv_init_x_ = 2e-3;
    conv_init_y_ = 2e-3;
    conv_init_phi_ = 2e-3;

    //滤波器相关参数
    min_particles_ = 50;
    max_particles_ = 200;
    alpha_slow_ = 0.001;
    alpha_fast_ = 0.1;
    pf_err_ =  0.01;   //kld_err 影响采样粒子数量？
    pf_z_ =    0.99;   //kld_z 
    resample_count_ = 0;
    resample_interval_ = 1; //每次滤波完毕后都进行重采样
    //ODOM相关参数
    //考虑使用图优化前端输出的位姿 置信度较高
    alpha1_ = 0.2;
    alpha2_ = 0.2;
    alpha3_ = 0.2;
    alpha4_ = 0.2;
    alpha5_ = 0.2;
    //LASER相关参数 似然域模型
    max_beams_ = 30; //保留100束激光点
    z_hit_ = 0.95;
    z_short_ = 0.1;
    z_max_ = 0.05;
    z_rand_ = 0.05;
    sigma_hit_ = 0.5;
    lambda_short_ = 0.1;
    laser_likelihood_max_dist_ = 0.5;

    //进行新一轮AMCL计算的移动向阈值判定
    d_thresh_ = 0.2;
    a_thresh_ = M_PI/6.0;
    laser_max_range_ = 8.0;
    laser_min_range_ = 0.0;

}
/**
 * @name: 
 * @des: 
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
AmclTracker::~AmclTracker()
{
    if (map_ != NULL)
    {
        map_free(map_);
        map_ = NULL;
    }
    if (pf_ != NULL)
    {
        pf_free( pf_ );
        pf_ = NULL;
    }
    
    delete odom_;
    odom_ = NULL;

    delete laser_;
    laser_ = NULL;
}

/**
 * @name: ConvertMap
 * @des:  转化栅格地图到AMCL地图格式
 * @param {TMapInfo} &tMapInfo
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
map_t* AmclTracker::ConvertMap(const TMapInfo *ptMapInfo)
{
    map_t* map = map_alloc();
    assert(map != NULL);

    map->size_x = ptMapInfo->iWidth;
    map->size_y = ptMapInfo->iHeight;
    map->scale = ptMapInfo->dResolution;
    map->origin_x = ptMapInfo->dOriginXOffset + (map->size_x / 2) * map->scale;
    map->origin_y = ptMapInfo->dOriginYOffset + (map->size_y / 2) * map->scale;
    // Convert to player format
    map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
    assert(map != NULL);

    for(int i = 0; i < map->size_x * map->size_y; i++)
    {
        //栅格为空 无障碍 概率值 0-60
        if (ptMapInfo->pCostMapData[i] >= 0 && 
            ptMapInfo->pCostMapData[i] <= 60)
        {
            map->cells[i].occ_state = -1;
        }
        //栅格占用 障碍物 概率值60-100
        else if (ptMapInfo->pCostMapData[i] >= 60 
            && ptMapInfo->pCostMapData[i] <= 100)
        {
            map->cells[i].occ_state = +1;
        }
        //栅格未知 未探测
        else
        {
            map->cells[i].occ_state = 0;
        }

    }

  return map;
}

/**
 * @name: UniformPoseGenerator
 * @des:  在地图空白位置随机采样 为重采样注入新粒子
 * @param {void*} arg
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
std::vector<std::pair<int,int> > AmclTracker::free_space_indices;
pf_vector_t AmclTracker::UniformPoseGenerator(void* arg)
{
    map_t* map = (map_t*)arg;

    unsigned int rand_index = drand48() * free_space_indices.size();
    std::pair<int,int> free_point = free_space_indices[rand_index];
    pf_vector_t p;
    p.v[0] = MAP_WXGX(map, free_point.first);
    p.v[1] = MAP_WYGY(map, free_point.second);
    p.v[2] = drand48() * 2 * M_PI - M_PI;

    return p;
}

/**
 * @name: Init
 * @des:  AMCL位姿跟踪器 初始化 在加载新的栅格地图后调用
 * @param {TMapInfo} *tMapInfo
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AmclTracker::Init(const TMapInfo *ptMapInfo)
{

//重置栅格地图数据--------------------------------------
    if (map_ != NULL)
    {
        map_free( map_ );
    }
    map_ = ConvertMap(ptMapInfo);

    //设置空白栅格索引序列 存储后在后续重采样注入新的粒子时用
    free_space_indices.resize(0);
    for(int i = 0; i < map_->size_x; i++)
    {
        for(int j = 0; j < map_->size_y; j++)
        {
            if(map_->cells[MAP_INDEX(map_,i,j)].occ_state == -1)
            {
                free_space_indices.push_back(std::make_pair(i,j));
            }
        }
    }

//重新初始化概率分布对象------------------------------------------------
    if( pf_ != NULL )
    {
        pf_free( pf_ );
        pf_ = NULL;
    }	
    pf_ = pf_alloc(min_particles_, max_particles_,
                alpha_slow_, alpha_fast_,
                (pf_init_model_fn_t)AmclTracker::UniformPoseGenerator,
                (void *)map_);
    pf_set_selective_resampling(pf_, false); //关闭选择重采样方法
    //TODO 概率密度对象相关参数调整
    pf_->pop_err = pf_err_;
    pf_->pop_z = pf_z_;
    resample_count_ = 0;
    //重新初始化概率分布对象
    pf_vector_t pf_init_pose_mean = pf_vector_zero();
    pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
    pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
    pf_init_ = false; //初始位姿未给定

//重置传感器模型实例------------------------------------------------------
    //Odometry双轮差速模型
    if (odom_ != NULL)
    {
        delete odom_;
        odom_ = NULL;
    }
    odom_ = new AMCLOdom();
    odom_->SetModel(ODOM_MODEL_DIFF, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_ );

    //Laser似然域模型
    if (laser_ != NULL)
    {
        delete laser_;
        laser_ = NULL;
    }
    laser_ = new AMCLLaser(max_beams_, map_);
    //初始化雷达的安装位姿 相对于BASELINK
    pf_vector_t laser_pose_v;
    Rigid3d r3TransB2L;
    poAgvTf_->GetTransform("base_link", "laser", r3TransB2L); //获取雷达中心相对于AGV基座标系的变换
    TAgvPose2D tPose2d = ToAgvPose2D(r3TransB2L);
    laser_pose_v.v[0] = tPose2d.x;
    laser_pose_v.v[1] = tPose2d.y;
    laser_pose_v.v[2] = 0;    //安装角度不处理 后续在收到雷达帧后 再做处理
    printf("INFO: AMCL Laser Module Init X[%f] Y[%f]\n", laser_pose_v.v[0], laser_pose_v.v[1]);
    laser_->SetLaserPose(laser_pose_v);
    //SetModelLikelihoodField 设置似然域模型并预算地图每个栅格到最近占用栅格的距离 耗时较高
    laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_,
                                laser_likelihood_max_dist_);

    return 0;
}
/**
 * @name: Reset
 * @des:  重置粒子滤波器 在确定机器人的全局位姿后调用 (SLAM后端回调)
 * @param {Rigid3d} &r3GlobalOrg
 * @param {TMapInfo} *tMapInfo
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AmclTracker::Reset(const Rigid3d &r3GlobalOrg, const Rigid3d &r3LocalOrg)
{
    int s32Ret;

    if (this->reset_conut ++ > 5)
    {
        return 0;
    }

    TAgvPose2D tGlobalPose2D = ToAgvPose2D(r3GlobalOrg);
    TAgvPose2D tLocalPose2D = ToAgvPose2D(r3LocalOrg);
    //2根据当前位姿设定期望
    pf_vector_t pf_init_pose_mean = pf_vector_zero();
    pf_init_pose_mean.v[0] = tGlobalPose2D.x;
    pf_init_pose_mean.v[1] = tGlobalPose2D.y;
    pf_init_pose_mean.v[2] = tGlobalPose2D.phi;
    pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
    //3设定当前位姿的协方差 假设XY PHI相互独立
    pf_init_pose_cov.m[0][0] = conv_init_x_;
    pf_init_pose_cov.m[1][1] = conv_init_y_;
    pf_init_pose_cov.m[2][2] = conv_init_phi_;
    //4重新初始化PF 锁定操作
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if(map_ != NULL ) 
        {
            pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
        }
        else
        {
            return -1;
        }
        //设定里程计原始位姿
        pf_odom_pose_.v[0] = tLocalPose2D.x;
        pf_odom_pose_.v[1] = tLocalPose2D.y;
        pf_odom_pose_.v[2] = tLocalPose2D.phi;
        pf_init_ = true;
    }
    printf("INFO: AMCL PF Reset With Local X[%f]-Y[%f]-PHI[%f]\n", 
        pf_init_pose_mean.v[0], 
        pf_init_pose_mean.v[1], 
        pf_init_pose_mean.v[2]);

    return 0;
}
/**
 * @name: LaserScanProcess
 * @des:  收到一帧雷达数据进行定位计算
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AmclTracker::LaserScanProcess(const Rigid3d &r3LocalCarto,
    const TAgvLaserScanMsg *ptLaserScan,
    Rigid3d &r3GlobalAmcl)
{
    TAgvPose2D tPose2dCarto = ToAgvPose2D(r3LocalCarto);
    pf_vector_t pose;
    pose.v[0] = tPose2dCarto.x;
    pose.v[1] = tPose2dCarto.y;
    pose.v[2] = tPose2dCarto.phi;

    Rigid3d r3TransM2O;
    poAgvTf_->GetTransform("map", "odom", r3TransM2O);
    r3GlobalAmcl = r3TransM2O * r3LocalCarto;
    
    std::lock_guard<std::mutex> lock(mutex_); //加锁

    if(!pf_init_) //未初始化 则使用Carto给的位姿
    {
        return -1;
    }

    pf_vector_t delta = pf_vector_zero();
    delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
    delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
    delta.v[2] = AngleDiff(pose.v[2], pf_odom_pose_.v[2]);

//阈值判定----------------------------------------------------------
    bool update = fabs(delta.v[0]) > d_thresh_ ||
                fabs(delta.v[1]) > d_thresh_ ||
                fabs(delta.v[2]) > a_thresh_;

    if (!update)
    {
        return -1;
    }

//1使用ODOM数据计算先验分布---------------------------------------------------
    AMCLOdomData odata;
    odata.pose = pose;
    odata.delta = delta;
    odom_->UpdateAction(pf_, (AMCLSensorData*)&odata);

//2使用LASER数据计算后验分布--------------------------------------------------
    AMCLLaserData ldata;
    ldata.sensor = laser_;
    ldata.range_count = ptLaserScan->ranges_size;
    //TODO 这里激光雷达的安装相对于底盘姿态变换为单位矩阵
    double angle_min = ptLaserScan->angle_min;
    double angle_increment = ptLaserScan->angle_increment;
    if(laser_max_range_ > 0.0)
    {
        ldata.range_max = //雷达的最大监测距离和用户定义的最大监测距离阈值取小
        std::min(ptLaserScan->range_max, (float)laser_max_range_);
    }
    else
    {
        ldata.range_max = ptLaserScan->range_max;
    }
    double range_min;
    if(laser_min_range_ > 0.0)
    {
        range_min =      //雷达的最小监测距离和用户定义的最小监测距离阈值取大
        std::max(ptLaserScan->range_min, (float)laser_min_range_);
    }
    else
    {
        range_min = ptLaserScan->range_min;
    }
    //为range数据分配内存 局部对象AMCLLaserData在函数返回时会析构 同时释放该内存
    ldata.ranges = new double[ldata.range_count][2];
    for(int i = 0; i < ldata.range_count; i++)
    {
        // amcl doesn't (yet) have a concept of min range.  So we'll map short
        // readings to max range.
        if(ptLaserScan->ranges[i] <= range_min)
        {
            ldata.ranges[i][0] = ldata.range_max;
        }
        else
        {
            ldata.ranges[i][0] = ptLaserScan->ranges[i];
        }
        ldata.ranges[i][1] = angle_min + (i * angle_increment);
    }
    //计算后验分布
    laser_->UpdateSensor(pf_, (AMCLSensorData*)&ldata);

    //更新K-1时刻里程计数据
    pf_odom_pose_ = pose;

//3粒子重采样--------------------------------------------------------------------
    bool resampled = false;
    if(!(++resample_count_ % resample_interval_))
    {
      pf_update_resample(pf_);
      resampled = true;
    }
    pf_sample_set_t* set = pf_->sets + pf_->current_set;
    printf("INFO: AMCL Num Samples After Resample: %d\n", set->sample_count);

//4估计机器人位姿 KDTree直方图MLE-------------------------------------------------
    TAgvPose2D tEstPose2D;
    double max_weight = 0.0;
    int max_weight_hyp = -1;
    std::vector<amcl_hyp_t> hyps;
    hyps.resize(pf_->sets[pf_->current_set].cluster_count);
    for(int hyp_count = 0; hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
    {
        double weight;
        pf_vector_t pose_mean;
        pf_matrix_t pose_cov;
        //计算每个Cluster的粒子集权重和
        if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov))
        {
            printf("INFO: AMCL Couldn't Get Stats On Cluster %d\n", hyp_count);
            break;
        }
        hyps[hyp_count].weight = weight;
        hyps[hyp_count].pf_pose_mean = pose_mean;
        hyps[hyp_count].pf_pose_cov = pose_cov;
        //统计最大的粒子权重和最大的Cluster
        if(hyps[hyp_count].weight > max_weight)
        {
            max_weight = hyps[hyp_count].weight;
            max_weight_hyp = hyp_count;
        }
    }

    if(max_weight > 0.0)
    {
        printf("INFO: AMCL Max Weight Pose: %.3f %.3f %.3f\n",
            hyps[max_weight_hyp].pf_pose_mean.v[0],
            hyps[max_weight_hyp].pf_pose_mean.v[1],
            hyps[max_weight_hyp].pf_pose_mean.v[2]);
        
        tEstPose2D.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
        tEstPose2D.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
        tEstPose2D.phi = hyps[max_weight_hyp].pf_pose_mean.v[2];

        r3GlobalAmcl = ToRigid3d(tEstPose2D);
        return 0;
    }
    else
    {
        printf("INFO: AMCL Caculate Global Pose Failed!\n");
        return -1;
    }

    return 0;

}
