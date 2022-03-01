#include "factory_manager.h"

#include <string>

#include <std_srvs/Trigger.h>
#include <nist_gear/KittingShipment.h>
#include <nist_gear/AssemblyShipment.h>

FactoryManager::FactoryManager(ros::NodeHandle* nodehandle):
  m_nh{*nodehandle}
{
  // Subscribers
  m_order_subscriber = m_nh.subscribe("/ariac/orders", 10, &FactoryManager::order_callback, this); 
  m_busy_subscriber = m_nh.subscribe("/worker/busy", 50, &FactoryManager::busy_callback, this); 

  // Publishers
  m_kitting_publisher = m_nh.advertise<nist_gear::KittingShipment>("/factory_manager/kitting_task", 10); 
  m_assembly_publisher = m_nh.advertise<nist_gear::AssemblyShipment>("/factory_manager/assembly_task", 10); 

  // All workers are free at start
  for (auto& worker: m_workers) {
    m_busy_state[worker] = false; 
  }
}

void FactoryManager::order_callback(const nist_gear::Order::ConstPtr& msg)
{
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 

  // emplace_back directly create object inside vector which is more efficient
  // Use unique_ptr to store msg as resources of this vector 
  m_orders.emplace_back(std::make_unique<nist_gear::Order>(*msg)); 

}

void FactoryManager::busy_callback(const my_ariac::Busy& msg){
  m_busy_state[msg.id] = msg.state; 
}

void FactoryManager::start_competition()
{
  std::string service_name = "/ariac/start_competition"; 

  static auto client = m_nh.serviceClient<std_srvs::Trigger>(service_name); 

  if (!client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }

  std_srvs::Trigger srv; 

  if (client.call(srv)) {
    ROS_INFO("%s", srv.response.message.c_str()); 
  }
  else {
    ROS_ERROR("Failed to call %s", service_name.c_str()); 
  }
 
}

void FactoryManager::end_competition()
{
  std::string service_name = "/ariac/end_competition"; 

  static auto client = m_nh.serviceClient<std_srvs::Trigger>(service_name); 

  if (!client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }

  std_srvs::Trigger srv; 

  if (client.call(srv)) {
    ROS_INFO("%s", srv.response.message.c_str()); 
  }
  else {
    ROS_ERROR("Failed to call %s", service_name.c_str()); 
  }

} 

bool FactoryManager::get_order()
{
  ros::Rate wait_rate(1); 
  // Wait for order for 10 seconds
  int count = 10; 
  while (m_orders.empty() && ros::ok()) {
    ros::spinOnce(); 
    ROS_INFO("Waiting orders for %ds...", count);
    if (count < 0) {
      ROS_INFO("No order.");
      return false; 
    }
    count--; 
    wait_rate.sleep(); 
  }
  return true; 
}

void FactoryManager::plan() 
{
  // Lock to prevent adding new orders when assigning tasks
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 
  ROS_INFO_STREAM("Orders: " << m_orders.size()); 

  for (auto &order: m_orders) {
    for (auto &shipment: order->kitting_shipments) {
      this->assign_kitting_task(shipment); 
    }

    for (auto &shipment: order->assembly_shipments) {
      this->assign_assembly_task(shipment); 
    }
  }

  m_orders.clear(); 
}

void FactoryManager::assign_kitting_task(nist_gear::KittingShipment& shipment)
{
  nist_gear::KittingShipment msg; 
  msg = shipment; 

  ROS_INFO("kitting task"); 
  m_kitting_publisher.publish(msg); 
}

void FactoryManager::assign_assembly_task(nist_gear::AssemblyShipment& shipment){
  nist_gear::AssemblyShipment msg; 
  msg = shipment; 

  ROS_INFO("assembly task"); 
  m_assembly_publisher.publish(msg); 
}

bool FactoryManager::work_done()
{
  for (const auto& worker_busy_state: m_busy_state) {
    if (worker_busy_state.second) {
      ROS_INFO("Waiting for worker: %s", worker_busy_state.first.c_str());
      return false; 
    }
  }
  return true; 
}

