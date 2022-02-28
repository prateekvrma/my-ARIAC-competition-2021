#ifndef STATION_H
#define STATION_H

#include <string>
#include <vector>
#include <memory>
#include <mutex>

#include <ros/ros.h>

#include <nist_gear/AssemblyShipment.h>
#include <std_msgs/String.h>

#include "sensors.h"

class Station {
  public:
    Station(ros::NodeHandle* nodehandle, const std::string& id); 
    bool get_order();
    void plan();
    void execute_tasks(const nist_gear::AssemblyShipment* task_ptr);
    void submit_shipment(const std::string& shipment_type); 

  private:
    // Callback functions
    void competition_state_callback(const std_msgs::String::ConstPtr& msg);
    void task_callback(const nist_gear::AssemblyShipment::ConstPtr& msg); 

    void publish_busy_state(); 

    // Contructor arguments
    ros::NodeHandle m_nh; 
    std::string m_id; 

    // Publisher
    ros::Publisher m_busy_publisher; 

    // Subscribers
    ros::Subscriber m_competition_state_subscriber; 
    ros::Subscriber m_task_subscriber; 

    // Subscribe info storage
    std::string m_competition_state; 
    std::vector<std::unique_ptr<nist_gear::AssemblyShipment>> m_tasks; 

    // Sensors
    std::string m_logical_camera_id = "logical_camera_station"; 
    LogicalCamera m_logical_camera; 

    // Mutex
    std::unique_ptr<std::mutex> m_mutex_ptr = std::make_unique<std::mutex>(); 

}; 

#endif


