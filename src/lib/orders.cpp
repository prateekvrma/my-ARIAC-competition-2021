#include "orders.h"

#include <ariac_group1/GetParts.h> 

OrderInfo::OrderInfo(const std::string& id,
                     const nist_gear::Order::ConstPtr& order_ptr,
                     const int priority_value):
  order_id{id},
  order{std::make_unique<nist_gear::Order>(*order_ptr)},
  priority{priority_value}
{
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
}

Orders::Orders(ros::NodeHandle* nodehandle):
  m_nh{*nodehandle}
{
  m_get_shipment_priority_service = m_nh.advertiseService("/orders/get_shipment_priority", &Orders::get_shipment_priority, this); 
  m_order_subscriber = m_nh.subscribe("/ariac/orders", 10, &Orders::order_callback, this); 
}

bool Orders::get_order()
{
  ros::Rate wait_rate(1); 

  // Wait for order for 10 seconds
  int wait_counter = 10; 

  while (ros::ok()) {
    ros::spinOnce(); 

    if (this->has_order()) {

      // check validity of new orders
      for (auto& new_order_id: m_new_orders_id) {
        bool valid = this->check_order(new_order_id); 
      }

      break; 

    }

    this->check_insufficient_orders(); 

    if (wait_counter < 0) {
      // ROS_INFO("No order.");
      return false; 
    }

    // check insufficient order

    // ROS_INFO("Waiting orders for %ds...", wait_counter);
    wait_counter--; 
    
    wait_rate.sleep(); 
  }

  return true; 
}

std::vector<std::string> Orders::get_new_orders_id() 
{
  return m_new_orders_id; 
}

void Orders::clear_new_orders_id()
{
  m_new_orders_id.clear(); 
}

void Orders::order_callback(const nist_gear::Order::ConstPtr& msg)
{
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 

  int priority = 0; 
  // High priority order if it is not the first order
  if (orders_record.size() > 0) {
    ROS_INFO("High-priority order is announced"); 
    priority = 1; 
  }

  // New order
  m_new_orders_id.push_back(msg->order_id); 

  // Order id for searching orders record
  m_orders_id.push_back(msg->order_id); 

  // Store all the order
  orders_record[msg->order_id] = std::make_unique<OrderInfo>(msg->order_id, msg, priority); 

  for (auto &shipment: msg->kitting_shipments) {
    m_shipments_id.push_back(shipment.shipment_type); 
    m_shipments_priority[shipment.shipment_type] = priority;  
  }

  for (auto &shipment: msg->assembly_shipments) {
    m_shipments_id.push_back(shipment.shipment_type); 
    m_shipments_priority[shipment.shipment_type] = priority;  
  }
}

bool Orders::has_order() 
{
  return not m_new_orders_id.empty(); 
}

void Orders::check_insufficient_orders() 
{
  for (auto& order_id: m_orders_id) {
    auto& order_info = orders_record[order_id]; 

    if (order_info->state == OrderState::Checked){
      // Check again after 20s passed since last check
      if ((ros::Time::now().toSec() - order_info->last_check) > 20) {

        bool valid = this->check_order(order_id); 
        if (valid == false) {
          ROS_INFO("Insufficient parts to complete %s", order_id.c_str()); 
          order_info->last_check = ros::Time::now().toSec();
          order_info->insufficient = true; 
        }
      }
    }
  }
}

bool Orders::check_order(const std::string& order_id)
{
  std::string service_name = "/sensor_manager/get_parts"; 

  static auto client = m_nh.serviceClient<ariac_group1::GetParts>(service_name); 

  if (!client.exists()) {
    ROS_INFO("Waiting for sensor manager...");
    client.waitForExistence();
    ROS_INFO("Sensor information is now ready.");
  }

  // Store the check time
  orders_record[order_id]->last_check = ros::Time::now().toSec();

  ROS_INFO("Checking %s", order_id.c_str()); 
  ROS_INFO("----------"); 
  // true if all parts in order is exists
  bool order_valid = true; 

  for (auto& wanted_type: orders_record[order_id]->wanted_type_count) {
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

  orders_record[order_id]->valid = order_valid;
  orders_record[order_id]->state = OrderState::Checked;

  return order_valid; 
}

bool Orders::get_shipment_priority(ariac_group1::GetShipmentPriority::Request &req, 
                                   ariac_group1::GetShipmentPriority::Response &res) 
{
  if (m_shipments_priority.count(req.shipment_type)) {
    res.priority = m_shipments_priority[req.shipment_type]; 
  }
  else {
    // error, no shipment type
    res.priority = -1; 
  }

  return true; 
}


