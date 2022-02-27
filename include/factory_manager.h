#ifndef FACTORY_MANAGER_H
#define FACTORY_MANAGER_H

#include <vector>
#include <memory>
#include <mutex>
#include <map>

#include <ros/ros.h>

#include <nist_gear/Order.h>

class FactoryManager{
  public:
    FactoryManager(ros::NodeHandle* nodehandle); 
    void order_callback(const nist_gear::Order::ConstPtr & msg); 

    void start_competition(); 
    void end_competition(); 

    bool get_order();
    void plan(); 

    bool work_done(); 

  private: 
    void assign_kitting_task(nist_gear::KittingShipment &shipment);
    void assign_assembly_task(nist_gear::AssemblyShipment &shipment);

   const std::vector<std::string> m_workers{"agv1", "agv2", "agv3", "agv4",
                                            "as1", "as2", "as3", "as4"}; 

    ros::NodeHandle m_nh; 
    ros::Subscriber m_order_subscriber;
    std::vector<std::unique_ptr<nist_gear::Order>> m_orders; 
    std::unique_ptr<std::mutex> m_mutex_ptr = std::make_unique<std::mutex>(); 

    ros::Publisher m_kitting_publisher; 
    ros::Publisher m_assembly_publisher; 

    ros::Subscriber m_check_busy_subscriber;
    std::map<std::string, bool> m_busy_state; 

}; 

#endif 


