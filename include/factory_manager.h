#ifndef FACTORY_MANAGER_H
#define FACTORY_MANAGER_H

#include <vector>
#include <memory>
#include <mutex>
#include <map>

#include <ros/ros.h>

#include <ariac_group1/Busy.h>

#include "orders.h"

class FactoryManager {
  public:
    FactoryManager(ros::NodeHandle* nodehandle); 

    void run_competition(); 

  private: 

    void busy_callback(const ariac_group1::Busy& msg); 

    void start_competition(); 

    void end_competition(); 

    void plan(); 

    bool work_done(); 
    /**
     * @Brief Assigning kitting tasks to AGVs 
     *
     * @Param shipment
     */
    void assign_kitting_task(nist_gear::KittingShipment& shipment);

    /**
     * @Brief Assigning assembly tasks to AssemblyStations
     *
     * @Param shipment
     */
    void assign_assembly_task(nist_gear::AssemblyShipment& shipment);

    Orders m_orders; 

    /**
     * @Brief The id of all the controllable machine in ARIAC 
     *
     */
    const std::vector<std::string> m_workers{"agv1", "agv2", "agv3", "agv4",
                                             "as1", "as2", "as3", "as4"}; 

    /**
     * @Brief node handle for AssemblyStation
     * 
     */
    ros::NodeHandle m_nh; 

    ros::Time m_start_time; 

    /**
     * @Brief Publisher to assign kitting task to AGVs  
     *
     */
    ros::Publisher m_kitting_publisher; 

    /**
     * @Brief Publisher to assign assembly task to AssemblyStation
     *
     */
    ros::Publisher m_assembly_publisher; 

    /**
     * @Brief Subscriber forr getting working status of all the worker machine 
     *
     */
    ros::Subscriber m_busy_subscriber;

    /**
     * @Brief The working status of all the worker machine  
     *        Busy is true if the machien still have task in its task queue
     *
     */
    std::map<std::string, bool> m_busy_state; 

    /**
     * @brief mutex to control the accessibility of the orders vector
     * 
     */
    std::unique_ptr<std::mutex> m_mutex_ptr = std::make_unique<std::mutex>(); 

}; 

#endif 


