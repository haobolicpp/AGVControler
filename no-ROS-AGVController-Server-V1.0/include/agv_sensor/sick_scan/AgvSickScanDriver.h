/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-08-14 11:17:05
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-08-14 16:13:18
 * @FilePath: /agv_controller/include/agv_sensor/sick_scan/AgvSickScanDriver.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
//Agv用西客激光雷达的驱动接口 


#ifndef AGVSICKSCANDRIVER_H
#define AGVSICKSCANDRIVER_H

#include <string>
#include <vector>
#include <cmath>

// Package
#include <SickSafetyscanners.h>
#include <datastructure/CommSettings.h>
#include <datastructure/FieldData.h>

#include "AgvCtrl.h"
#include "agv_type.h"

using namespace sick;

class AgvSickScanDriver
{

public:
    bool m_initialised;
    std::shared_ptr<sick::SickSafetyscanners> m_device;
    sick::datastructure::CommSettings m_communication_settings;
    boost::asio::ip::address_v4 m_interface_ip;

    std::string m_frame_id = "laser";
    double m_time_offset;
    double m_range_min;
    double m_range_max;
    double m_frequency_tolerance      = 0.1;
    double m_expected_frequency       = 20.0;
    double m_timestamp_min_acceptable = -1.0;
    double m_timestamp_max_acceptable = 1.0;
    double m_min_intensities          = 0.0; /*!< min intensities for laser points */

    bool m_use_sick_angles;
    float m_angle_offset;
    bool m_use_pers_conf;

    void readTypeCodeSettings();
    void receivedUDPPacket(const datastructure::Data& data);
    int CreateLaserScanMessage(const sick::datastructure::Data& data, 
            TAgvLaserScanMsg &tAgvLaserScanMsg);

    AgvCtrl *pt_agv_ctrl;

private:
    /* data */
public:
    AgvSickScanDriver(AgvCtrl *pt_agv_ctrl);
    ~AgvSickScanDriver();

public:
    int Init();
    int Start();

};





#endif // AGVSICKSCANDRIVER_H