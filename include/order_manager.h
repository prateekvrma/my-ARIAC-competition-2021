#ifndef ORDER_MANAGER_H
#define ORDER_MANAGER_H

#include <vector>
#include <memory>
#include <mutex>
#include <map>
#include <string>

#include <ros/ros.h>

#include <nist_gear/Order.h>

enum class OrderState{New, Checked, Finish}; 

class OrderInfo {
  public: 
    OrderInfo(const std::string& id,
              const nist_gear::Order::ConstPtr& order_ptr); 

    std::string order_id;  

    std::unique_ptr<nist_gear::Order> order; 

    std::map<std::string, int> wanted_type_count; 

    double last_check = -1; 

    OrderState state = OrderState::New; 

    // is order temporary insufficient
    bool valid = false; 

    // insufficient if order is invalid for a duration
    bool insufficient = false; 
}; 

class OrderManager {
  public:
    OrderManager(ros::NodeHandle* nodehandle);  

    bool get_order(); 
    std::vector<std::string> get_new_orders_id(); 
    void clear_new_orders_id(); 

    std::map<std::string, std::unique_ptr<OrderInfo>> orders_record; 

  private:
    void order_callback(const nist_gear::Order::ConstPtr& msg); 

    bool has_order(); 
    void check_insufficient_orders(); 
    bool check_order(const std::string& order_id); 

    ros::NodeHandle m_nh; 
    ros::Subscriber m_order_subscriber;

    std::vector<std::string> m_new_orders_id; 
    std::vector<std::string> m_orders_id; 

    std::unique_ptr<std::mutex> m_mutex_ptr = std::make_unique<std::mutex>(); 
}; 

#endif 
