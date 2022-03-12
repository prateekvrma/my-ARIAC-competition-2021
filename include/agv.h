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

/**
 * @brief AGV class to handle the assigned tasks to the agv
 * 
 */
class AGV {
  public: 
    AGV(ros::NodeHandle* nodehandle, const std::string& id); 

    /**
     * @brief check if AGV recieves the order
     * 
     * @return true 
     * @return false 
     */
    bool get_order();

    /**
     * @brief the task which AGV plan to do 
     * 
     */
    void plan();

    /**
     * @brief perform the shipment action
     * 
     * @param task_ptr 
     */
    void execute_tasks(const nist_gear::KittingShipment* task_ptr);

    /**
     * @brief call the service to allow AGV to submit kitting shipment to assembly station
     * 
     * @param shipment_type 
     * @param station_id 
     */
    void submit_shipment(const std::string& shipment_type,
                         const std::string& station_id); 
    /**
     * @brief call the service to allow AGV to go to the assembly station
     * 
     * @param station_id 
     */
    void to_as(const std::string& station_id); 

  private:

    /**
     * @brief callback function for getting the state of AGV
     * 
     * @param msg 
     */
    void state_callback(const std_msgs::String::ConstPtr& msg);
    
    /**
     * @brief callback function for getting the state of the station
     * 
     * @param msg 
     */
    void station_callback(const std_msgs::String::ConstPtr& msg);

    /**
     * @brief callback function for getting the competition state
     * 
     * @param msg 
     */
    void competition_state_callback(const std_msgs::String::ConstPtr& msg);

    /**
     * @brief callback function for getting the type of task is assigned to AGV
     * 
     * @param msg 
     */
    void task_callback(const nist_gear::KittingShipment::ConstPtr& msg); 

    /**
     * @brief publish msg to m_busy_publisher topic
     * 
     */
    void publish_busy_state(); 

    /**
     * @brief node handle for AGV
     * 
     */
    ros::NodeHandle m_nh; 

    /**
     * @brief assign id to AGV
     * 
     */
    std::string m_id; 

    /**
     * @brief create a publisher to show that AGV is busy
     * 
     */
    ros::Publisher m_busy_publisher; 

    /**
     * @brief create a subscriber for recieving the state of AGV
     * 
     */
    ros::Subscriber m_state_subscriber; 

    /**
     * @brief create a subscriber for recieving the current position of the AGV 
     * 
     */
    ros::Subscriber m_station_subscriber; 

    /**
     * @brief create a subsriber for checking the competion state
     * 
     */
    ros::Subscriber m_competition_state_subscriber; 

    /**
     * @brief create a subscriber for checking the assigned task
     * 
     */
    ros::Subscriber m_task_subscriber; 

    /**
     * @brief the message received from m_state_subscriber 
     * 
     */
    std::string m_state; 

    /**
     * @brief the message received from m_station_subscriber
     * 
     */
    std::string m_station; 

    /**
     * @brief the message received from m_competition_state
     * 
     */
    std::string m_competition_state;  

    /**
     * @brief the vector which stores the sequence of task
     * 
     */
    std::vector<std::unique_ptr<nist_gear::KittingShipment>> m_tasks; 

    /**
     * @brief the prefix id of quality control sensor
     * 
     */
    std::string m_quality_control_sensor_id = "quality_control_sensor_"; 

    /**
     * @brief the object of quality control sensor
     * 
     */
    LogicalCamera m_quality_control_sensor; 

    /**
     * @brief mutex to control the accessibility of the tasks vector
     * 
     */
    std::unique_ptr<std::mutex> m_mutex_ptr = std::make_unique<std::mutex>(); 

}; 
/**
 * @brief check if the station is valid
 * 
 * @param agv 
 * @param station 
 * @return true 
 * @return false 
 */
bool valid_station(const std::string& agv, const std::string& station);

#endif


