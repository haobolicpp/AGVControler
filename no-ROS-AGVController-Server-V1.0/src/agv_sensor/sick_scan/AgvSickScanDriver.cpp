

#include "tlog.h"
#include "agv_cmd.h"
#include "agv_type.h"
#include "AgvCtrl.h"

#include "AgvSickScanDriver.h"

/*!
 * \brief Converts degrees to radians.
 * \param deg Degrees to convert.
 * \return To radians converted degrees.
 */
inline float degToRad(float deg)
{
  return deg * M_PI / 180.0f;
}

/*!
 * \brief Converts radians to degrees.
 * \param rad Input radians to convert
 * \return To degrees converted radians
 */
inline float radToDeg(float rad)
{
  return rad * 180.0f / M_PI;
}

/**
 * @brief AgvSickScanDriver构造
 * @param 
 * @return
 */
AgvSickScanDriver::AgvSickScanDriver(AgvCtrl *pt_agv_ctrl)
{
    this->pt_agv_ctrl = pt_agv_ctrl;
}

/**
 * @brief AgvSickScanDriver析构
 * @param 
 * @return
 */
AgvSickScanDriver::~AgvSickScanDriver()
{

}

/**
 * @brief AgvSickScanDriver初始化
 * @param 
 * @return
 */
int AgvSickScanDriver::Init()
{
    std::string sensor_ip_adress = "192.168.192.100";
    m_communication_settings.setSensorIp(sensor_ip_adress);

    std::string host_ip_adress = "192.168.192.101";
    m_communication_settings.setHostIp(host_ip_adress);

    std::string interface_ip_adress = "0.0.0.0";
    m_interface_ip = boost::asio::ip::address_v4::from_string(interface_ip_adress);

    int host_udp_port = 0;
    m_communication_settings.setHostUdpPort(host_udp_port);


    int channel = 0;
    m_communication_settings.setChannel(channel);

    bool enabled = true;
    m_communication_settings.setEnabled(enabled);

    int skip = 0;
    m_communication_settings.setPublishingFrequency(skip + 1);

    // float angle_start = -1.5708;
    // float angle_end = 1.5708;
    float angle_start = -3*M_PI/4;
    float angle_end = 3*M_PI/4;
    m_angle_offset = -90.0;

    //m_private_nh.getParam("frequency_tolerance", m_frequency_tolerance);
    //m_private_nh.getParam("expected_frequency", m_expected_frequency);
    //m_private_nh.getParam("timestamp_min_acceptable", m_timestamp_min_acceptable);
    //m_private_nh.getParam("timestamp_max_acceptable", m_timestamp_max_acceptable);

    // Included check before calculations to prevent rounding errors while calculating
    if (angle_start == angle_end)
    {
    m_communication_settings.setStartAngle(radToDeg(0));
    m_communication_settings.setEndAngle(radToDeg(0));
    }
    else
    {
    m_communication_settings.setStartAngle(radToDeg(angle_start) - m_angle_offset);
    m_communication_settings.setEndAngle(radToDeg(angle_end) - m_angle_offset);
    }
    m_time_offset = 0;
    //m_private_nh.getParam("time_offset", m_time_offset);

    bool general_system_state = true;
    //m_private_nh.getParam("general_system_state", general_system_state);

    bool derived_settings = true;
    //m_private_nh.getParam("derived_settings", derived_settings);

    bool measurement_data = true;
    //m_private_nh.getParam("measurement_data", measurement_data);

    bool intrusion_data = true;
    //m_private_nh.getParam("intrusion_data", intrusion_data);

    bool application_io_data = true;
    //m_private_nh.getParam("application_io_data", application_io_data);

    m_communication_settings.setFeatures(
    general_system_state, derived_settings, measurement_data, intrusion_data, application_io_data);

    m_frame_id = "laser";
    //m_private_nh.getParam("frame_id", m_frame_id);
    //m_private_nh.getParam("use_persistent_config", m_use_pers_conf);
    //m_private_nh.getParam("min_intensities", m_min_intensities);


    m_communication_settings.setSensorTcpPort(2122);
    m_device = std::make_shared<sick::SickSafetyscanners>(
    boost::bind(&AgvSickScanDriver::receivedUDPPacket, this, _1),
    &m_communication_settings,
    m_interface_ip);
    m_device->run();
    readTypeCodeSettings();

    m_device->changeSensorSettings(m_communication_settings);

    return 0;
}

/**
 * @brief AgvSickScanDriver开始运行
 * @param 
 * @return
 */
int AgvSickScanDriver::Start()
{
    return 0;
}

/**
 * @brief AgvSickScanDriver读取雷达设备内部默认参数
 * @param 
 * @return
 */
void AgvSickScanDriver::readTypeCodeSettings()
{
    printf("Reading Type code settings\n");
    sick::datastructure::TypeCode type_code;
    m_device->requestTypeCode(m_communication_settings, type_code);
    m_communication_settings.setEInterfaceType(type_code.getInterfaceType());
    m_range_min = 0.1;
    m_range_max = 39.0;
    // m_range_max = type_code.getMaxRange();
}

/**
 * @brief AgvSickScanDriver异步接收Scan数据
 * @param 
 * @return
 */
double timeLaserDiff_ = 0;;
double timeLaserPre_ = 0;

void AgvSickScanDriver::receivedUDPPacket(const sick::datastructure::Data& data)
{
    int s32Ret;
    TAgvLaserScanMsg tAgvLaserScanMsg;
    // uint8_t buffer[20 * 1024];
    TRobotMsgHeader tAgvMsgHeader = {0};
    TRobotMsg tAgvMsgForSlammer = {0};

    uint32_t laserDataLength = 0;

    if (!data.getMeasurementDataPtr()->isEmpty() && !data.getDerivedValuesPtr()->isEmpty())
    {
        s32Ret = CreateLaserScanMessage(data, tAgvLaserScanMsg);
        if (s32Ret < 0)
        {
            printf("ERROR: CreateLaserScanMessage Falied\n");
            return;
        }


        TLinuxTime tTimeNow; 
        LinuxTimeNow(&tTimeNow);
        double timeLaserNow = (tTimeNow.tv_sec * 1e9 + tTimeNow.tv_nsec)/1e6;
        timeLaserDiff_ = timeLaserNow - timeLaserPre_;
        timeLaserPre_ = timeLaserNow;

        // printf("INFO: LaserScanMessage Generate with timeP[%f]-ranges[%d]-stime[%f]\n", 
        // timeLaserDiff_, tAgvLaserScanMsg.ranges_size, tAgvLaserScanMsg.scan_time);
        if (timeLaserDiff_ < 10.0)
        {
            tlog(TLOG_WARN, "Laser Scan Message generate period is too small with ranges[%d] period[%f]\n", 
            tAgvLaserScanMsg.ranges_size, timeLaserDiff_);
        }

        tAgvMsgHeader.u16_class = AGV_SLAM_C_SENSOR;
        tAgvMsgHeader.u16_type = AGV_SLAM_T_SENSOR_LASER;
        tAgvMsgHeader.s32_len = sizeof(TAgvLaserScanMsg);
        RobotMsgInit(&tAgvMsgForSlammer, tAgvMsgHeader, (uint8_t *)&tAgvLaserScanMsg);
        s32Ret = pt_agv_ctrl->poAgvSlammer->AsyncMsgPost(&tAgvMsgForSlammer);
        if (s32Ret < 0)
        {
            printf("ERROR: LaserScanMessage Post Falied with timeP[%f]\n", timeLaserDiff_);
            return;
        }
    }

}

/**
 * @brief AgvSickScanDriver转化原始Scan数据为标准雷达数据类型
 * @param 
 * @return
 */
// double laser_max = 0;
int AgvSickScanDriver::CreateLaserScanMessage(const sick::datastructure::Data& data, 
    TAgvLaserScanMsg &tAgvLaserScanMsg)
{
    TAgvTimeStamp tAgvTimeStamp = {0};
    //获取系统时间ns
    const std::chrono::nanoseconds now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                            std::chrono::system_clock::now().time_since_epoch());
    int64_t s64_now =  now.count();
    tAgvTimeStamp = fromNSec(s64_now);
    tAgvLaserScanMsg.header.stamp = tAgvTimeStamp;
    //header process
    snprintf(tAgvLaserScanMsg.header.frame_id, FRAME_IDS_MAX, "laser");
    // tAgvLaserScanMsg.header.frame_id = m_frame_id;
    // tAgvLaserScanMsg.header.stamp.sec = 0; //todo
    // tAgvLaserScanMsg.header.stamp.nsec = 0;

    std::vector<sick::datastructure::ScanPoint> scan_points =
    data.getMeasurementDataPtr()->getScanPointsVector();

    uint32_t num_scan_points = scan_points.size();
    if (num_scan_points >= LASER_RANGES_MAX)
    {
        return -1;
    }
    // assert(num_scan_points < LASER_RANGES_MAX);

    tAgvLaserScanMsg.angle_min = degToRad(data.getDerivedValuesPtr()->getStartAngle() + m_angle_offset);
    tAgvLaserScanMsg.angle_max = degToRad(data.getMeasurementDataPtr()
                     ->getScanPointsVector()
                     .at(data.getMeasurementDataPtr()->getScanPointsVector().size() - 1)
                     .getAngle() + m_angle_offset);
    tAgvLaserScanMsg.angle_increment = degToRad(data.getDerivedValuesPtr()->getAngularBeamResolution());

    boost::posix_time::microseconds time_increment =
    boost::posix_time::microseconds(data.getDerivedValuesPtr()->getInterbeamPeriod());
    tAgvLaserScanMsg.time_increment = time_increment.total_microseconds() * 1e-6;

    boost::posix_time::milliseconds scan_time =
    boost::posix_time::milliseconds(data.getDerivedValuesPtr()->getScanTime());
    tAgvLaserScanMsg.scan_time = scan_time.total_microseconds() * 1e-6;
    tAgvLaserScanMsg.range_min = m_range_min;
    tAgvLaserScanMsg.range_max = m_range_max;
    tAgvLaserScanMsg.ranges_size = num_scan_points;
    tAgvLaserScanMsg.intensities_size = num_scan_points;
    // tAgvLaserScanMsg.ranges.resize(num_scan_points);
    // tAgvLaserScanMsg.intensities.resize(num_scan_points);
    for (uint32_t i = 0; i < num_scan_points; ++i)
    {
        const sick::datastructure::ScanPoint scan_point = scan_points.at(i);
        // Filter for intensities
        if (m_min_intensities < static_cast<double>(scan_point.getReflectivity()))
        {
            tAgvLaserScanMsg.ranges[i] = static_cast<float>(scan_point.getDistance()) *
                data.getDerivedValuesPtr()->getMultiplicationFactor() * 1e-3; // mm -> m
            // Set values close to/greater than max range to infinity according to REP 117
            // https://www.ros.org/reps/rep-0117.html
            if (tAgvLaserScanMsg.ranges[i] >= (0.999 * m_range_max))
            {
                tAgvLaserScanMsg.ranges[i] = std::numeric_limits<double>::infinity();
            }
            // else
            // {
            //     laser_max = tAgvLaserScanMsg.ranges[i] > laser_max ? tAgvLaserScanMsg.ranges[i] : laser_max;
            // }
        }
        else
        {
            tAgvLaserScanMsg.ranges[i] = std::numeric_limits<double>::infinity();
        }
        tAgvLaserScanMsg.intensities[i] = static_cast<float>(scan_point.getReflectivity());
    }
    // printf("INFO: Laser Max %f\n", laser_max);

  return 0;
}