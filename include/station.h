#ifndef STATION_H
#define STATION_H

// ros
#include <ros/ros.h>

// standard library
#include <string>
#include <vector>
#include <memory>
#include <mutex>

// services and messages
#include <std_msgs/String.h>
// nist
#include <nist_gear/AssemblyShipment.h>

class Station {
  public:
    Station(ros::NodeHandle* nodehandle, const std::string& id); 
    
    void submit_shipment(const std::string& shipment_type); 

  private:
    
    ros::NodeHandle m_nh; 

    ros::ServiceClient m_submit_shipment_client; 

    std::string m_id; 

    std::unique_ptr<std::mutex> m_mutex_ptr = std::make_unique<std::mutex>(); 

}; 

#endif


