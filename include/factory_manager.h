#ifndef FACTORY_MANAGER_H
#define FACTORY_MANAGER_H

#include <vector>
#include <memory>

#include <ros/ros.h>

#include <nist_gear/Order.h>

class FactoryManager{
  public:
    FactoryManager(ros::NodeHandle* nodehandle); 
    void order_callback(const nist_gear::Order::ConstPtr & msg); 

    void start_competition(); 
    void end_competition(); 

    void plan(); 

  private: 
    //void assign_kitting_task();
    //void assign_assembly_task();

    ros::NodeHandle m_nh; 
    ros::Subscriber m_order_subscriber;
    std::vector<std::unique_ptr<nist_gear::Order>> m_orders; 
}; 

#endif 


