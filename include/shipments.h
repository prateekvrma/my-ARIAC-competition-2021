#ifndef SHIPMENTS_H
#define SHIPMENTS_H

// ros
#include <ros/ros.h>

// standard library
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <tuple>
#include <map>

// custom library
#include "constants.h"
#include "utility.h"

// services and messages
#include <std_msgs/String.h>

// nist
#include <nist_gear/KittingShipment.h>

// custom
#include <ariac_group1/PartTask.h>
#include <ariac_group1/PartsUnderCamera.h>


enum class ShipmentState{NOT_READY, READY, INSUFFICIENT,
                         HAS_REDUNDANT, HAS_WRONG_TYPE, HAS_WRONG_POSE, HAS_FLIP_PART, HAS_MISSING_PART, HAS_FAULTY,
                         POSTPONE, FINISH}; 

class ShipmentInfo {
  public: 
    ShipmentInfo(const std::string& id,
              const nist_gear::KittingShipment::ConstPtr& shipment_ptr); 

    std::string shipment_id;  
    std::unique_ptr<nist_gear::KittingShipment> shipment; 
    std::vector<nist_gear::Model> target_parts_in_world; 
    int unfinished_part_tasks; 
    int priority = 0; 
    ShipmentState state = ShipmentState::NOT_READY; 
};

class Shipments {
  public:
    Shipments(ros::NodeHandle* nodehandle);  
    void update_part_task_queue(std::vector<std::tuple<int, std::unique_ptr<ariac_group1::PartTask>>>& part_task_queue); 
    bool is_high_priority_alert(); 
    bool check_redundant(ariac_group1::PartTask& part_task, nist_gear::Model& wrong_part); 
    std::string check_shipment_parts(ariac_group1::PartTask& part_task, nist_gear::Model& wrong_part); 
    bool is_part_task_done(const ariac_group1::PartTask& part_task); 

    std::map<std::string, std::unique_ptr<ShipmentInfo>> shipments_record; 

  private: 
    void shipment_callback(const nist_gear::KittingShipment::ConstPtr& msg); 
    bool has_shipment(); 

    // ros
    ros::NodeHandle m_nh; 

    // ros subscriber
    ros::Subscriber m_shipment_subscriber;

    // ros service client
    ros::ServiceClient m_get_shipment_priority_client;
    ros::ServiceClient m_parts_under_camera_client; 

    std::vector<std::string> m_new_shipments_id; 
    std::vector<std::string> m_shipments_id; 
    std::vector<std::string> m_high_priorities_id; 

    bool m_high_priority_alert = false; 

    std::unique_ptr<std::mutex> m_mutex_ptr = std::make_unique<std::mutex>(); 
}; 

#endif 


