#ifndef FACTORY_MANAGER_H
#define FACTORY_MANAGER_H

#include <vector>

#include <ros/ros.h>

#include <nist_gear/Order.h>

#include "order_info.h"

class FactoryManager{
  public:
    FactoryManager(ros::NodeHandle* nodehandle); 
    void order_callback(const nist_gear::Order::ConstPtr & msg); 

    void start_competition(); 
    void end_competition(); 

  private: 
    ros::NodeHandle m_nh; 
    ros::Subscriber m_order_subscriber;
    std::vector<Order> m_orders; 
}; 

#endif 


