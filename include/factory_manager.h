#ifndef FACTORY_MANAGER_H
#define FACTORY_MANAGER_H

#include <vector>
#include <memory>
#include <mutex>
#include <map>

#include <ros/ros.h>

#include <nist_gear/Order.h>
#include <my_ariac/Busy.h>

class FactoryManager{
  public:
    FactoryManager(ros::NodeHandle* nodehandle); 
    void start_competition(); 
    void end_competition(); 

    bool get_order();
    void plan(); 

    bool work_done(); 

  private: 
    // Callback functions
    void order_callback(const nist_gear::Order::ConstPtr & msg); 
    void busy_callback(const my_ariac::Busy & msg); 

    // Publisher functions
    void assign_kitting_task(nist_gear::KittingShipment &shipment);
    void assign_assembly_task(nist_gear::AssemblyShipment &shipment);

    // Workers control by factory manager
    const std::vector<std::string> m_workers{"agv1", "agv2", "agv3", "agv4",
                                             "as1", "as2", "as3", "as4"}; 

    // Constructor argument
    ros::NodeHandle m_nh; 

    // Publishers 
    ros::Publisher m_kitting_publisher; 
    ros::Publisher m_assembly_publisher; 

    // Subscribers
    ros::Subscriber m_order_subscriber;
    ros::Subscriber m_busy_subscriber;

    // Subscribe info storage
    std::vector<std::unique_ptr<nist_gear::Order>> m_orders; 
    std::map<std::string, bool> m_busy_state; 

    // Mutex
    std::unique_ptr<std::mutex> m_mutex_ptr = std::make_unique<std::mutex>(); 

}; 

#endif 


