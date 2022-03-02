/**
 * @file station.h
 * @author Bo-Shiang Wang (bwang24@umd.edu), Chang-Hong Chen (markchen@umd.edu), Prateek Verma (verma@umd.edu), Sparsh Jaiswal (sjaiswal@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2022-03-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */

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

/**
 * @brief Class member function of the Station Node.
 * 
 */
class Station {
  public:
    Station(ros::NodeHandle* nodehandle, const std::string& id); 

    /**
     * @brief Check if AGV recieves the order
     * 
     * @return true 
     * @return false 
     */
    bool get_order();

    /**
     * @brief Function to define the plan of action.
     * 
     */
    void plan();

    /**
     * @brief Function to execute each task.
     * 
     * @param task_ptr 
     */
    void execute_tasks(const nist_gear::AssemblyShipment* task_ptr);

    /**
     * @brief Function to submit shipment.
     * 
     * @param shipment_type 
     */
    void submit_shipment(const std::string& shipment_type); 

  private:

    /**
     * @brief Callback function to access the state of the competition
     * 
     * @param msg 
     */
    void competition_state_callback(const std_msgs::String::ConstPtr& msg);

    /**
     * @brief Callback function to access the task status during competition.
     * 
     * @param msg 
     */
    void task_callback(const nist_gear::AssemblyShipment::ConstPtr& msg); 

    /**
     * @brief Function to publish message for busy state.
     * 
     */
    void publish_busy_state(); 

    /**
     * @brief node handle for AssemblyStation
     * 
     */
    ros::NodeHandle m_nh; 

    /**
     * @brief assign id to AssemblyStation
     * 
     */
    std::string m_id; 

    /**
     * @brief create a publisher to show that AssemblyStation is busy
     * 
     */
    ros::Publisher m_busy_publisher; 

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
     * @brief the message received from m_competition_state
     * 
     */
    std::string m_competition_state; 

    /**
     * @brief the vector which stores the sequence of task
     * 
     */
    std::vector<std::unique_ptr<nist_gear::AssemblyShipment>> m_tasks; 

    /**
     * @brief the prefix id of logical camera sensor
     * 
     */
    std::string m_logical_camera_id = "logical_camera_station"; 

    /**
     * @brief the object of logical camera sensor
     * 
     */
    LogicalCamera m_logical_camera; 

    /**
     * @brief mutex to control the accessibility of the tasks vector
     * 
     */
    std::unique_ptr<std::mutex> m_mutex_ptr = std::make_unique<std::mutex>(); 

}; 

#endif


