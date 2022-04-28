#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <vector>
#include <string>
#include <memory>
#include <map>
#include <mutex>

#include <ros/ros.h>

#include <ariac_group1/PartInfo.h>
#include <ariac_group1/GetParts.h>
#include <ariac_group1/IsFaulty.h>
#include <ariac_group1/IsShipmentReady.h>
#include <ariac_group1/PartsInCamera.h>
#include <ariac_group1/IsPartPicked.h>
#include <ariac_group1/GetPartPosition.h>
#include <ariac_group1/CheckQualitySensor.h>
#include <ariac_group1/GetVacancyPose.h>

#include "sensors.h"
#include "utility.h"

class SensorManager {
  public:
    SensorManager(ros::NodeHandle* nodehandle); 

    void update_parts(); 
    void show_database(); 
    void check_blackout(); 

  private:
    void print_bins_occupancy(); 

    bool get_parts(ariac_group1::GetParts::Request &req, 
                   ariac_group1::GetParts::Response &res); 

    bool is_faulty(ariac_group1::IsFaulty::Request &req, 
                   ariac_group1::IsFaulty::Response &res); 

    bool is_shipment_ready(ariac_group1::IsShipmentReady::Request &req, 
                           ariac_group1::IsShipmentReady::Response &res);  

    bool parts_in_camera(ariac_group1::PartsInCamera::Request &req,
                         ariac_group1::PartsInCamera::Response &res); 

    bool is_part_picked(ariac_group1::IsPartPicked::Request &req,
                        ariac_group1::IsPartPicked::Response &res); 

    bool get_part_position(ariac_group1::GetPartPosition::Request &req,
                           ariac_group1::GetPartPosition::Response &res); 

    bool check_quality_sensor(ariac_group1::CheckQualitySensor::Request &req,
                              ariac_group1::CheckQualitySensor::Response &res); 

    bool get_vacancy_pose(ariac_group1::GetVacancyPose::Request &req,
                          ariac_group1::GetVacancyPose::Response &res); 

    std::string convert_id_to_internal(std::string global_id); 
 
    ros::ServiceServer m_get_parts_service; 
    ros::ServiceServer m_is_faulty_service;
    ros::ServiceServer m_is_shipment_ready_service;
    ros::ServiceServer m_parts_in_camera_service;
    ros::ServiceServer m_is_part_picked_service;
    ros::ServiceServer m_get_part_position_service;
    ros::ServiceServer m_check_quality_sensor_service;
    ros::ServiceServer m_get_vacancy_pose_service;

    const std::vector<std::string> m_logical_cameras{// AGV parking spot at Assembly Station
                                                     // "as1_1", "as2_1", "as1_2", "as2_2",
                                                     // "as3_3", "as4_3", "as3_4", "as4_4", 
                                                     // Briefcase
                                                     // "bfc1", "bfc2", "bfc3", "bfc4",  
                                                     // Kitting Station
                                                     "ks1", "ks2", "ks3", "ks4",
                                                     // Belt,
                                                     // "belt",
                                                     // Bins
                                                     "bins0", "bins1"}; 

    const std::vector<std::string> m_quality_sensors{// Quality sensors on AGV 
                                                     "quality_control_sensor_1",
                                                     "quality_control_sensor_2",
                                                     "quality_control_sensor_3",
                                                     "quality_control_sensor_4"}; 

    std::map<std::string, std::unique_ptr<LogicalCamera>> m_logical_cameras_dict; 

    std::map<std::string, std::unique_ptr<LogicalCamera>> m_quality_sensors_dict; 

    std::map<std::string, std::vector<std::unique_ptr<ariac_group1::PartInfo>>> m_parts_database; 


    const std::vector<std::string> m_bins_id{"bin1", "bin2", "bin3", "bin4", "bin5", "bin6", "bin7", "bin8"}; 
    std::map<std::string, std::vector<bool>> bins_occupancy; 

    ros::NodeHandle m_nh; 

    bool m_sensors_blackout = false; 

    std::unique_ptr<std::mutex> m_mutex_ptr = std::make_unique<std::mutex>(); 

}; 

#endif 


