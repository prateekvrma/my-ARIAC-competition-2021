#ifndef FACTORY_MANAGER_H
#define FACTORY_MANAGER_H

// ros
#include <ros/ros.h>

// standard library
#include <vector>
#include <memory>
#include <mutex>
#include <map>

// custom library
#include "orders.h"

// services and messages
// nist
#include <ariac_group1/GetCompetitionTime.h>

// custom
#include <ariac_group1/Busy.h>

class FactoryManager {
  public:
    FactoryManager(ros::NodeHandle* nodehandle); 

    void run_competition(); 

  private: 

    // callback
    void busy_callback(const ariac_group1::Busy& msg); 

    // flow
    void start_competition(); 
    void end_competition(); 
    void plan(); 


    // order managing
    void assign_kitting_task(nist_gear::KittingShipment& shipment);
    void assign_assembly_task(nist_gear::AssemblyShipment& shipment);

    bool get_competition_time(ariac_group1::GetCompetitionTime::Request &req,
                              ariac_group1::GetCompetitionTime::Response &res); 

    bool work_done(); 

    // ros
    ros::NodeHandle m_nh; 

    // ros publisher
    ros::Publisher m_kitting_publisher; 
    ros::Publisher m_assembly_publisher; 

    // ros subscriber
    ros::Subscriber m_busy_subscriber;

    // ros service server
    ros::ServiceServer m_get_competition_time_service; 

    Orders m_orders; 

    const std::vector<std::string> m_workers{"agv1", "agv2", "agv3", "agv4",
                                             "as1", "as2", "as3", "as4"}; 

    // states
    ros::Time m_start_time; 
    std::map<std::string, bool> m_busy_state; 

    std::unique_ptr<std::mutex> m_mutex_ptr = std::make_unique<std::mutex>(); 
}; 

#endif 


