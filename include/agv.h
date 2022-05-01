#ifndef AGV_H
#define AGV_H

#include <string>
#include <vector>
#include <memory>
#include <mutex>

#include <ros/ros.h>

#include <std_msgs/String.h>

/**
 * @brief AGV class to handle the assigned tasks to the agv
 * 
 */
class AGV {
  public: 
    AGV(ros::NodeHandle* nodehandle, const std::string& id); 

    void submit_shipment(const std::string& shipment_type,
                         const std::string& station_id); 

    void to_as(const std::string& station_id); 

  private:

    void state_callback(const std_msgs::String::ConstPtr& msg);
    void station_callback(const std_msgs::String::ConstPtr& msg);

    ros::NodeHandle m_nh; 

    std::string m_id; 

    std::string m_kitting_station_id = "ks"; 

    ros::Subscriber m_state_subscriber; 
    ros::Subscriber m_station_subscriber; 


    ros::ServiceClient m_submit_shipment_client; 

    std::string m_state; 

    std::string m_station; 

    std::unique_ptr<std::mutex> m_mutex_ptr = std::make_unique<std::mutex>(); 

}; 

bool valid_station(const std::string& agv, const std::string& station);

#endif


