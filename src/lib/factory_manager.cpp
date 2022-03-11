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

  // All Logical cameras in the environment
  
  // AGV parking spot at assembly stations
  std::string as1_1 = "logical_camera_as1_1"; 
  m_logical_cameras[as1_1] = std::make_unique<LogicalCamera>(nodehandle, as1_1); 

  std::string as2_1 = "logical_camera_as2_1"; 
  m_logical_cameras[as2_1] = std::make_unique<LogicalCamera>(nodehandle, as2_1); 

  std::string as1_2 = "logical_camera_as1_2"; 
  m_logical_cameras[as1_2] = std::make_unique<LogicalCamera>(nodehandle, as1_2); 

  std::string as2_2 = "logical_camera_as2_2"; 
  m_logical_cameras[as2_2] = std::make_unique<LogicalCamera>(nodehandle, as2_2); 

  std::string as3_3 = "logical_camera_as3_3"; 
  m_logical_cameras[as3_3] = std::make_unique<LogicalCamera>(nodehandle, as3_3); 

  std::string as4_3 = "logical_camera_as4_3"; 
  m_logical_cameras[as4_3] = std::make_unique<LogicalCamera>(nodehandle, as4_3); 

  std::string as3_4 = "logical_camera_as3_4"; 
  m_logical_cameras[as3_4] = std::make_unique<LogicalCamera>(nodehandle, as3_4); 

  std::string as4_4 = "logical_camera_as4_4"; 
  m_logical_cameras[as4_4] = std::make_unique<LogicalCamera>(nodehandle, as4_4); 

  // Briefcase
  std::string bfc1 = "logical_camera_bfc1"; 
  m_logical_cameras[bfc1] = std::make_unique<LogicalCamera>(nodehandle, bfc1); 

  std::string bfc2 = "logical_camera_bfc2"; 
  m_logical_cameras[bfc2] = std::make_unique<LogicalCamera>(nodehandle, bfc2); 

  std::string bfc3 = "logical_camera_bfc3"; 
  m_logical_cameras[bfc3] = std::make_unique<LogicalCamera>(nodehandle, bfc3); 

  std::string bfc4 = "logical_camera_bfc4"; 
  m_logical_cameras[bfc4] = std::make_unique<LogicalCamera>(nodehandle, bfc4); 

  // Kitting station
  std::string ks1 = "logical_camera_ks1"; 
  m_logical_cameras[ks1] = std::make_unique<LogicalCamera>(nodehandle, ks1); 

  std::string ks2 = "logical_camera_ks2"; 
  m_logical_cameras[ks2] = std::make_unique<LogicalCamera>(nodehandle, ks2); 

  std::string ks3 = "logical_camera_ks3"; 
  m_logical_cameras[ks3] = std::make_unique<LogicalCamera>(nodehandle, ks3); 

  std::string ks4 = "logical_camera_ks4"; 
  m_logical_cameras[ks4] = std::make_unique<LogicalCamera>(nodehandle, ks4); 

  // Belt
  std::string belt = "logical_camera_belt"; 
  m_logical_cameras[belt] = std::make_unique<LogicalCamera>(nodehandle, belt); 

  // Bins
  std::string bins0 = "logical_camera_bins0"; 
  m_logical_cameras[bins0] = std::make_unique<LogicalCamera>(nodehandle, bins0); 

  std::string bins1 = "logical_camera_bins1"; 
  m_logical_cameras[bins1] = std::make_unique<LogicalCamera>(nodehandle, bins1); 

}

void FactoryManager::order_callback(const nist_gear::Order::ConstPtr& msg)
{
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 

  // emplace_back directly create object inside vector which is more efficient
  // Use unique_ptr to store msg as resources of this vector 
  m_orders.emplace_back(std::make_unique<nist_gear::Order>(*msg)); 

}

void FactoryManager::busy_callback(const ariac_group1::Busy& msg){
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

  m_kitting_publisher.publish(msg); 
}

void FactoryManager::assign_assembly_task(nist_gear::AssemblyShipment& shipment){
  nist_gear::AssemblyShipment msg; 
  msg = shipment; 

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

