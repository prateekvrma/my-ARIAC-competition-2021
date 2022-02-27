#ifndef AGV_H
#define AGV_H

#include <string>
#include <vector>
#include <memory>
#include <mutex>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <nist_gear/KittingShipment.h>
#include <std_msgs/String.h>

#include "sensors.h"

class AGV {
  public: 
    AGV(ros::NodeHandle* nodehandle, const std::string &id); 

    void state_callback(const std_msgs::String::ConstPtr & msg);
    void station_callback(const std_msgs::String::ConstPtr & msg);
    void task_callback(const nist_gear::KittingShipment::ConstPtr &msg); 
    void competition_state_callback(const std_msgs::String::ConstPtr &msg);
    bool get_order();
    void plan();
    void execute_tasks(const nist_gear::KittingShipment *task_ptr);

    void submit_shipment(const std::string &shipment_type,
                         const std::string &station_id); 

    void to_as(const std::string &station_id); 

  private:
    void publish_busy_state(); 

    ros::NodeHandle m_nh; 
    std::string m_id; 
    ros::Subscriber m_state_subscriber; 
    ros::Subscriber m_station_subscriber; 

    std::string m_state; 
    std::string m_station; 

    std::string m_quality_control_sensor_id = "quality_control_sensor_"; 
    LogicalCamera m_quality_control_sensor; 

    ros::Subscriber m_task_subscriber; 
    std::vector<std::unique_ptr<nist_gear::KittingShipment>> m_tasks; 
    std::unique_ptr<std::mutex> m_mutex_ptr = std::make_unique<std::mutex>(); 

    ros::Publisher m_busy_publisher; 
    ros::Subscriber m_competition_state_subscriber; 
    std::string m_competition_state;  
}; 

bool valid_station(const std::string &agv, const std::string &station);

#endif


