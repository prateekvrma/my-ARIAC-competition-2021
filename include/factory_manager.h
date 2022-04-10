#ifndef FACTORY_MANAGER_H
#define FACTORY_MANAGER_H

#include <vector>
#include <memory>
#include <mutex>
#include <map>

#include <ros/ros.h>

#include <nist_gear/Order.h>
#include <ariac_group1/Busy.h>

#include "sensors.h"

/**
 * @Brief The state of the order  
 */
enum class OrderState{New, Checked, Finsih}; 

/**
 * @Brief Store information about an order   
 */
class OrderInfo {
  public: 
    OrderInfo(const std::string& id,
              const nist_gear::Order::ConstPtr& order_ptr); 

    /**
     * @Brief The id of an order
     */
    std::string order_id;  

    /**
     * @Brief The order content 
     */
    std::unique_ptr<nist_gear::Order> order; 

    /**
     * @Brief The last time this order has been checked 
     *        =1 mean not checked yet
     */
    double last_check = -1; 

    /**
     * @Brief The state of the order 
     *        New: new recieved order
     *        Checked: parts in order have been checked 
     *        Finish: the order has been submitted
     */
    OrderState state = OrderState::New; 

    /**
     * @Brief true if all parts are in environment  
     */
    bool valid = false; 
}; 

class FactoryManager {
  public:
    FactoryManager(ros::NodeHandle* nodehandle); 

    void run_competition(); 

  private: 

    /**
     * @Brief Subscriber callback function for order information. 
     *
     * @Param msg
     */
    void order_callback(const nist_gear::Order::ConstPtr& msg); 

    /**
     * @Brief Susbscriber callback function for recieving busy state 
     *        from every worker machine.
     *
     * @Param msg
     */
    void busy_callback(const ariac_group1::Busy& msg); 

    void start_competition(); 

    void end_competition(); 

    bool get_order();

    void plan(); 

    /**
     * @Brief Check if every worker machine is busy   
     *
     * @Returns Is there any worker machine still working    
     */
    bool work_done(); 
    /**
     * @Brief Check if all parts in an order is in environment
     *
     * @Param order_id
     *
     * @Returns true if all parts are in envrironment  
     */
    bool check_order(const std::string& order_id); 

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
     * @Brief Subscirber for getting orders from ARIAC 
     *
     */
    ros::Subscriber m_order_subscriber;

    /**
     * @Brief Subscriber forr getting working status of all the worker machine 
     *
     */
    ros::Subscriber m_busy_subscriber;

    /**
     * @Brief New order recived 
     *
     */
    std::vector<std::string> m_new_orders; 

    /**
     * @Brief Id of all the past orders   
     */
    std::vector<std::string> m_orders_id; 

    /**
     * @Brief Store all the orders information by id  
     */
    std::map<std::string, std::unique_ptr<OrderInfo>> m_orders_record; 


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


