#ifndef AGV_H
#define AGV_H

#include <string>

#include <ros/ros.h>

class AGV {
  public: 
    AGV(ros::NodeHandle* nodehandle, const std::string &id); 

    void submit_shipment(const std::string &shipment_type,
                         const std::string &station_id); 

    void to_as(const std::string &station_id); 

  private:
    ros::NodeHandle m_nh; 
    std::string m_id; 
}; 

#endif


