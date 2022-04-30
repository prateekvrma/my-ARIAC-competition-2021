#include "shipments.h"

#include <ariac_group1/GetShipmentPriority.h>

ShipmentInfo::ShipmentInfo(const std::string& id,
                           const nist_gear::KittingShipment::ConstPtr& shipment_ptr):
  shipment_id{id},
  shipment{std::make_unique<nist_gear::KittingShipment>(*shipment_ptr)}
{
  unfinished_part_tasks = shipment->products.size(); 
}

Shipments::Shipments(ros::NodeHandle* nodehandle):
  m_nh{*nodehandle}
{
  m_shipment_subscriber = m_nh.subscribe("/factory_manager/kitting_task", 10, &Shipments::shipment_callback, this); 

  m_get_shipment_priority_client = 
      m_nh.serviceClient<ariac_group1::GetShipmentPriority>("/orders/get_shipment_priority"); 
  m_get_shipment_priority_client.waitForExistence();
}

void Shipments::shipment_callback(const nist_gear::KittingShipment::ConstPtr& msg)
{
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 
  // add tasks to task vector
  m_new_shipments_id.push_back(msg->shipment_type); 
  m_shipments_id.push_back(msg->shipment_type); 
  shipments_record[msg->shipment_type] = std::make_unique<ShipmentInfo>(msg->shipment_type, msg); 

  ariac_group1::GetShipmentPriority get_shipment_priority_srv; 
  get_shipment_priority_srv.request.shipment_type = msg->shipment_type;
  m_get_shipment_priority_client.call(get_shipment_priority_srv); 
  shipments_record[msg->shipment_type]->priority = get_shipment_priority_srv.response.priority; 

  if (shipments_record[msg->shipment_type]->priority != 0) {
    m_high_priorities_id.push_back(msg->shipment_type); 
  }

}

bool Shipments::has_shipment() 
{
  return not m_new_shipments_id.empty(); 
}

void Shipments::update_part_task_queue(std::vector<std::tuple<int, std::unique_ptr<ariac_group1::PartTask>>>& part_task_queue)
{
  ros::spinOnce(); 
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 

  if (this->has_shipment()) {
    for (auto& id: m_new_shipments_id) {
      for (auto& product: shipments_record[id]->shipment->products) {
        ariac_group1::PartTask part_task; 
        part_task.shipment_type = id; 
        part_task.part = product;  
        part_task.total_parts = shipments_record[id]->shipment->products.size(); 
        part_task.agv_id = shipments_record[id]->shipment->agv_id; 
        part_task.station_id = shipments_record[id]->shipment->station_id; 
        part_task.priority = shipments_record[id]->priority; 

        if (part_task.priority != 0) {
            bool all_finish = true; 
            for (auto& high_priority_id: m_high_priorities_id) {
                if (high_priority_id == id) {
                    continue; 
                }
                if (shipments_record[high_priority_id]->state != ShipmentState::FINISH) {
                    all_finish = false; 
                    break; 
                }
            }

            if (all_finish) {
                // only set alert when new shipment is high priority and unfinished shipments are all low priority
                ROS_INFO("High priority alert!!!"); 
                m_high_priority_alert = true; 
            }
        }
        part_task_queue.emplace_back(std::make_tuple(part_task.priority * Constants::PriorityWeight::Ratio::HIGH_PRIORITY,
                                                     std::make_unique<ariac_group1::PartTask>(part_task))); 
      }
    }

    m_new_shipments_id.clear(); 
  }
}

bool Shipments::is_high_priority_alert() 
{
    bool alert = m_high_priority_alert; 
    if (alert) {
      m_high_priority_alert = false; 
    }
    return alert; 
}

