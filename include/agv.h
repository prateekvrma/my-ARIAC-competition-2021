#ifndef AGV_H
#define AGV_H

#include <string>
#include <vector>
#include <memory>
#include <mutex>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <nist_gear/KittingShipment.h>

#include "sensors.h"

class AGV {
  public: 
    AGV(ros::NodeHandle* nodehandle, const std::string& id); 

    bool get_order();
    void plan();
    void execute_tasks(const nist_gear::KittingShipment* task_ptr);
    void submit_shipment(const std::string& shipment_type,
                         const std::string& station_id); 
    void to_as(const std::string& station_id); 

  private:
    // callback functions
    void state_callback(const std_msgs::String::ConstPtr& msg);
    void station_callback(const std_msgs::String::ConstPtr& msg);
    void competition_state_callback(const std_msgs::String::ConstPtr& msg);
    void task_callback(const nist_gear::KittingShipment::ConstPtr& msg); 

    void publish_busy_state(); 

    // Constructor arguments
    ros::NodeHandle m_nh; 
    std::string m_id; 

    // Publisher
    ros::Publisher m_busy_publisher; 

    // Subscribers
    ros::Subscriber m_state_subscriber; 
    ros::Subscriber m_station_subscriber; 
    ros::Subscriber m_competition_state_subscriber; 
    ros::Subscriber m_task_subscriber; 

    // Subscribe info storage
    std::string m_state; 
    std::string m_station; 
    std::string m_competition_state;  
    std::vector<std::unique_ptr<nist_gear::KittingShipment>> m_tasks; 

    // Sensors
    std::string m_quality_control_sensor_id = "quality_control_sensor_"; 
    LogicalCamera m_quality_control_sensor; 

    // Mutex
    std::unique_ptr<std::mutex> m_mutex_ptr = std::make_unique<std::mutex>(); 

}; 

bool valid_station(const std::string& agv, const std::string& station);

#endif


