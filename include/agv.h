#ifndef AGV_H
#define AGV_H

#include <string>

#include <ros/ros.h>

#include <std_msgs/String.h>

#include "sensors.h"

class AGV {
  public: 
    AGV(ros::NodeHandle* nodehandle, const std::string &id); 

    void state_callback(const std_msgs::String::ConstPtr & msg);
    void station_callback(const std_msgs::String::ConstPtr & msg);

    void submit_shipment(const std::string &shipment_type,
                         const std::string &station_id); 

    void to_as(const std::string &station_id); 

  private:
    ros::NodeHandle m_nh; 
    std::string m_id; 
    ros::Subscriber m_state_subscriber; 
    ros::Subscriber m_station_subscriber; 

    std::string m_state; 
    std::string m_station; 

    std::string m_quality_control_sensor_id = "quality_control_sensor_"; 
    LogicalCamera m_quality_control_sensor; 

}; 

bool valid_station(const std::string &agv, const std::string &station);

#endif


