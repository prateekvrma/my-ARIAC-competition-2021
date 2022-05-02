#ifndef SHIPMENTS_H
#define SHIPMENTS_H

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <tuple>
#include <map>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <nist_gear/KittingShipment.h>
#include <ariac_group1/PartTask.h>
#include <ariac_group1/PartsUnderCamera.h>

#include "constants.h"
#include "utility.h"

enum class ShipmentState{NOT_READY, READY,
                         HAS_WRONG_TYPE, HAS_WRONG_POSE, HAS_FLIP_PART, HAS_MISSING_PART, HAS_FAULTY,
                         POSTPONE, FINISH}; 

class ShipmentInfo {
  public: 
    ShipmentInfo(const std::string& id,
              const nist_gear::KittingShipment::ConstPtr& shipment_ptr); 

    std::string shipment_id;  
    std::unique_ptr<nist_gear::KittingShipment> shipment; 
    int unfinished_part_tasks; 
    int priority = 0; 
    ShipmentState state = ShipmentState::NOT_READY; 
};

class Shipments {
  public:
    Shipments(ros::NodeHandle* nodehandle);  
    bool get_shipment(); 
    void update_part_task_queue(std::vector<std::tuple<int, std::unique_ptr<ariac_group1::PartTask>>>& part_task_queue); 
    bool is_high_priority_alert(); 
    std::string check_shipment_parts(ariac_group1::PartTask& part_task, nist_gear::Model& wrong_part); 

    std::map<std::string, std::unique_ptr<ShipmentInfo>> shipments_record; 

  private: 
    void shipment_callback(const nist_gear::KittingShipment::ConstPtr& msg); 
    bool has_shipment(); 
    bool is_part_task_done(const ariac_group1::PartTask& part_task); 

    ros::NodeHandle m_nh; 
    ros::Subscriber m_shipment_subscriber;

    ros::ServiceClient m_get_shipment_priority_client;
    ros::ServiceClient m_parts_under_camera_client; 

    std::vector<std::string> m_new_shipments_id; 
    std::vector<std::string> m_shipments_id; 
    std::vector<std::string> m_high_priorities_id; 

    bool m_high_priority_alert = false; 

    std::unique_ptr<std::mutex> m_mutex_ptr = std::make_unique<std::mutex>(); 
}; 

#endif 


