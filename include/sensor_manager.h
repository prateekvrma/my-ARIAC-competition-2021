#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <vector>
#include <string>
#include <memory>
#include <map>

#include <ros/ros.h>
#include <ariac_group1/PartInfo.h>

#include "sensors.h"
#include "utility.h"

class SensorManager {
  public:
    SensorManager(ros::NodeHandle* nodehandle); 

    void update_parts(); 
    void show_database(); 

  private:
    const std::vector<std::string> m_logical_cameras{// AGV parking spot at Assembly Station
                                                     "as1_1", "as2_1", "as1_2", "as2_2",
                                                     "as3_3", "as4_3", "as3_4", "as4_4", 
                                                     // Briefcase
                                                     "bfc1", "bfc2", "bfc3", "bfc4",  
                                                     // Kitting Station
                                                     "ks1", "ks2", "ks3", "ks4",
                                                     // Belt,
                                                     "belt",
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

    ros::NodeHandle m_nh; 

}; 

#endif 


