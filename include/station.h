#ifndef STATION_H
#define STATION_H

#include <string>
#include <vector>
#include <memory>
#include <mutex>

#include <ros/ros.h>

#include <nist_gear/AssemblyShipment.h>

class Station {
  public:
    Station(ros::NodeHandle* nodehandle, const std::string &id); 
    void submit_shipment(const std::string &shipment_type); 
    void task_callback(const nist_gear::AssemblyShipment::ConstPtr &msg); 
    void plan();
    void execute_tasks(const nist_gear::AssemblyShipment *task_ptr);

  private:
    ros::NodeHandle m_nh; 
    std::string m_id; 
    ros::Subscriber m_task_subscriber; 

    std::vector<std::unique_ptr<nist_gear::AssemblyShipment>> m_tasks; 
    std::unique_ptr<std::mutex> m_mutex_ptr = std::make_unique<std::mutex>(); 
}; 

#endif


