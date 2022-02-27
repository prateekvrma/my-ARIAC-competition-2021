#ifndef STATION_H
#define STATION_H

#include <string>

#include <ros/ros.h>

class Station {
  public:
    Station(ros::NodeHandle* nodehandle, const std::string &id); 
    void submit_shipment(const std::string &shipment_type); 

  private:
    ros::NodeHandle m_nh; 
    std::string m_id; 

}; 

#endif


