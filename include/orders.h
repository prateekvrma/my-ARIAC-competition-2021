#ifndef ORDERS_H
#define ORDERS_H

// ros
#include <ros/ros.h>

// standard library
#include <vector>
#include <memory>
#include <mutex>
#include <map>
#include <string>


// services and messages
// nist
#include <nist_gear/Order.h>

// custom 
#include <ariac_group1/GetShipmentPriority.h>

enum class OrderState{New, Checked, Finish}; 

class OrderInfo {
  public: 
    OrderInfo(const std::string& id,
              const nist_gear::Order::ConstPtr& order_ptr,
              const int priority_value); 

    std::string order_id;  
    std::unique_ptr<nist_gear::Order> order; 
    std::map<std::string, int> wanted_type_count; 
    double last_check = -1; 

    // is order temporary insufficient
    bool valid = false; 

    // insufficient if order is invalid for a duration
    bool insufficient = false; 

    int priority = 0; 
    OrderState state = OrderState::New; 
}; 

class Orders {
  public:
    Orders(ros::NodeHandle* nodehandle);  

    bool get_order(); 
    std::vector<std::string> get_new_orders_id(); 
    void clear_new_orders_id(); 

    std::map<std::string, std::unique_ptr<OrderInfo>> orders_record; 

  private:
    // callback
    void order_callback(const nist_gear::Order::ConstPtr& msg); 

    bool has_order(); 
    void check_insufficient_orders(); 
    bool check_order(const std::string& order_id); 

    bool get_shipment_priority(ariac_group1::GetShipmentPriority::Request &req, 
                               ariac_group1::GetShipmentPriority::Response &res); 

    // ros
    ros::NodeHandle m_nh; 

    // ros subscriber
    ros::Subscriber m_order_subscriber;

    // ros service server
    ros::ServiceServer m_get_shipment_priority_service; 

    std::vector<std::string> m_new_orders_id; 
    std::vector<std::string> m_orders_id; 
    std::vector<std::string> m_shipments_id; 

    std::map<std::string, int> m_shipments_priority; 

    std::unique_ptr<std::mutex> m_mutex_ptr = std::make_unique<std::mutex>(); 
}; 

#endif 
