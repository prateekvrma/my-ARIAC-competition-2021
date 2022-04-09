#include "factory_manager.h"

#include <string>
#include <set>

#include <std_srvs/Trigger.h>
#include <nist_gear/KittingShipment.h>
#include <nist_gear/AssemblyShipment.h>
#include <ariac_group1/GetParts.h> 


OrderInfo::OrderInfo(const std::string& id,
                     const nist_gear::Order::ConstPtr& order_ptr):
  order_id{id},
  order{std::make_unique<nist_gear::Order>(*order_ptr)}
{

}

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

  // High priority order if it is not the first order
  if (m_orders_record.size() > 0) {
    ROS_INFO("High-priority order is announced"); 
  }

  // New order
  m_new_orders.push_back(msg->order_id); 

  // Order id for searching orders record
  m_orders_id.push_back(msg->order_id); 

  // Store all the order
  m_orders_record[msg->order_id] = std::make_unique<OrderInfo>(msg->order_id,
                                                               msg); 
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
  while (m_new_orders.empty() && ros::ok()) {
    ros::spinOnce(); 
    if (count < 0) {
      ROS_INFO("No order.");
      return false; 
    }
    ROS_INFO("Waiting orders for %ds...", count);
    count--; 

    // Check for insufficient parts in order
    for (auto& order_id: m_orders_id) {
      auto& order_info = m_orders_record[order_id]; 

      // Check for orders that didn't pass the check
      if (order_info->state == OrderState::Checked && order_info->valid == false){

        // Check again after 20s passed since last check
        if ((ros::Time::now().toSec() - order_info->last_check) > 20) {
          bool valid = this->check_order(order_id); 
          if (valid == false) {
            ROS_INFO("Insufficient parts to complete %s", order_id.c_str()); 
            order_info->last_check = ros::Time::now().toSec();
          }
        }
      }
    }

    wait_rate.sleep(); 
  }
  return true; 
}

void FactoryManager::plan() 
{
  // Lock to prevent adding new orders when assigning tasks
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 

  for (auto& order_id: m_new_orders) {
    // Check if parts valid in order
    auto valid = this->check_order(order_id); 
    auto& order = m_orders_record[order_id]->order; 

    for (auto &shipment: order->kitting_shipments) {
      this->assign_kitting_task(shipment); 
    }

    for (auto &shipment: order->assembly_shipments) {
      this->assign_assembly_task(shipment); 
    }
  }

  m_new_orders.clear(); 
}

bool FactoryManager::check_order(const std::string& order_id)
{
  std::string service_name = "/sensor_manager/get_parts"; 

  static auto client = m_nh.serviceClient<ariac_group1::GetParts>(service_name); 

  if (!client.exists()) {
    ROS_INFO("Waiting for sensor manager...");
    client.waitForExistence();
    ROS_INFO("Sensor information is now ready.");
  }

  // Store the check time
  m_orders_record[order_id]->last_check = ros::Time::now().toSec();
  auto& order = m_orders_record[order_id]->order; 

  ROS_INFO("Checking %s", order_id.c_str()); 
  ROS_INFO("----------"); 
  // true if all parts in order is exists
  bool order_valid = true; 

  std::map<std::string, int> wanted_type_count; 
  // check every shipment in order
  for (auto &shipment: order->kitting_shipments) {

    for ( auto &product: shipment.products) {

      if (wanted_type_count.count(product.type)) {

        wanted_type_count[product.type]++; 

      }
      else {

        wanted_type_count[product.type] = 1; 

      }
    }
  }

  for (auto& wanted_type: wanted_type_count) {
    auto type = wanted_type.first; 
    auto type_count = wanted_type.second; 

    ariac_group1::GetParts get_parts_srv; 
    get_parts_srv.request.type = type; 

    if (client.call(get_parts_srv)) {

      auto parts_in_factory = get_parts_srv.response.parts_info.size(); 
      if (parts_in_factory < type_count) {
        order_valid = false; 
      } 

      ROS_INFO("Found %lu %s in factory, %s needs %d", parts_in_factory, type.c_str(), order_id.c_str(), type_count); 

    }
    else {

      ROS_ERROR("Failed to call %s", service_name.c_str()); 

    }
  }

  ROS_INFO("----------"); 

  m_orders_record[order_id]->state = OrderState::Checked; 
  m_orders_record[order_id]->valid = order_valid;

  return order_valid; 

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

