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
  for (auto& camera_id: m_logical_cameras) {
    std::string prefix = "logical_camera_"; 
    m_logical_cameras_dict[camera_id] = std::make_unique<LogicalCamera>(nodehandle, prefix += camera_id); 
  }

  // All quality sensors in the environment
  for (auto& camera_id: m_quality_sensors) {
    m_quality_sensors_dict[camera_id] = std::make_unique<LogicalCamera>(nodehandle, camera_id); 
  }

}

void FactoryManager::order_callback(const nist_gear::Order::ConstPtr& msg)
{
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 

  // emplace_back directly create object inside vector which is more efficient
  // Use unique_ptr to store msg as resources of this vector 
  if (not m_orders_record.empty()){
    ROS_INFO("High-priority order is announced"); 
  }

  m_new_orders.emplace_back(std::make_unique<nist_gear::Order>(*msg)); 
  //m_unchecked_orders.emplace_back(std::make_unique<nist_gear::Order>(*msg)); 
  m_orders_record[msg->order_id] = std::make_unique<nist_gear::Order>(*msg); 
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
  ros::Time checking_time = ros::Time::now(); 
  // Wait for order for 10 seconds
  int count = 10; 
  while (m_new_orders.empty() && ros::ok()) {
    ros::spinOnce(); 
    if (count < 0) {
      ROS_INFO("No order.");
      return false; 
    }
    ROS_INFO("Waiting orders for %ds...", count);
    count--; 
    //checking_time = ros::Time::now(); 
    //this->check_order(); 
    wait_rate.sleep(); 
  }
  return true; 
}

void FactoryManager::plan() 
{
  // Lock to prevent adding new orders when assigning tasks
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 
  //ROS_INFO_STREAM("Orders: " << m_new_orders.size()); 

  for (auto &order: m_new_orders) {
    this->check_order(order->order_id); 
    for (auto &shipment: order->kitting_shipments) {
      this->assign_kitting_task(shipment); 
    }

    for (auto &shipment: order->assembly_shipments) {
      this->assign_assembly_task(shipment); 
    }
  }

  m_new_orders.clear(); 
}

void FactoryManager::check_order(const std::string& order_id)
{
  auto order = *m_orders_record[order_id]; 
  order_check_time[order_id] = ros::Time::now(); 
  ROS_INFO("Checking %s", order_id.c_str()); 
  ROS_INFO("----------"); 
  // check every shipment in order
  for (auto &shipment: order.kitting_shipments) {
    // check every product in shipment
    for ( auto &product: shipment.products) {
      int parts_count = 0; 
      // check every camera to see if product exists
      for (auto &camera_id: m_logical_cameras){
        parts_count += m_logical_cameras_dict[camera_id]->find_parts(product.type); 
      }
      if (parts_count == 0) {
        ROS_INFO("No %s in factory", product.type.c_str()); 
      }else{
        ROS_INFO("Found %d %s in factory", parts_count, product.type.c_str()); 
      }
    }
  }
  ROS_INFO("----------"); 

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

