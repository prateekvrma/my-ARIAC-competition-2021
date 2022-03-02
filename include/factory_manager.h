#ifndef FACTORY_MANAGER_H
#define FACTORY_MANAGER_H

#include <vector>
#include <memory>
#include <mutex>
#include <map>

#include <ros/ros.h>

#include <nist_gear/Order.h>
#include <my_ariac/Busy.h>

class FactoryManager {
  public:
    FactoryManager(ros::NodeHandle* nodehandle); 

    /**
     * @Brief Call service to start the competition
     *
     */
    void start_competition(); 

    /**
     * @Brief Call service to end the competition
     *
     */
    void end_competition(); 

    /**
     * @Brief Waiting for orders in a loop
     *
     * @Returns Order exists or not  
     */
    bool get_order();

    /**
     * @Brief Plan for the orders.
     *        Seperate orders into kitting and assembly tasks,
     *        then assign them to the AGVs, and AssemblyStations.
     */
    void plan(); 

    /**
     * @Brief Check if every worker machine is busy   
     *
     * @Returns Is there any worker machine still working    
     */
    bool work_done(); 

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
    void busy_callback(const my_ariac::Busy& msg); 

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

    // Mutex to control the accessibility of the tasks vector
    std::unique_ptr<std::mutex> m_mutex_ptr = std::make_unique<std::mutex>(); 

}; 

#endif 


