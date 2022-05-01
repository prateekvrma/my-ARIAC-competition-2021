#include "assembly_shipments.h"

#include <ariac_group1/GetShipmentPriority.h>

AssemblyShipmentInfo::AssemblyShipmentInfo(const std::string& id,
                           const nist_gear::AssemblyShipment::ConstPtr& shipment_ptr):
  shipment_id{id},
  shipment{std::make_unique<nist_gear::AssemblyShipment>(*shipment_ptr)}
{
  unfinished_part_tasks = shipment->products.size(); 
}

AssemblyShipments::AssemblyShipments(ros::NodeHandle* nodehandle):
  m_nh{*nodehandle}
{
  m_shipment_subscriber = m_nh.subscribe("/factory_manager/assembly_task", 10, &AssemblyShipments::shipment_callback, this); 

  m_get_shipment_priority_client = 
      m_nh.serviceClient<ariac_group1::GetShipmentPriority>("/orders/get_shipment_priority"); 
  m_get_shipment_priority_client.waitForExistence();

  m_parts_under_camera_client = 
      m_nh.serviceClient<ariac_group1::PartsUnderCamera>("/sensor_manager/parts_under_camera"); 
  m_parts_under_camera_client.waitForExistence();

}

void AssemblyShipments::shipment_callback(const nist_gear::AssemblyShipment::ConstPtr& msg)
{
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 
  // add tasks to task vector
  m_new_shipments_id.push_back(msg->shipment_type); 
  m_shipments_id.push_back(msg->shipment_type); 
  shipments_record[msg->shipment_type] = std::make_unique<AssemblyShipmentInfo>(msg->shipment_type, msg); 

  // ariac_group1::GetShipmentPriority get_shipment_priority_srv; 
  // get_shipment_priority_srv.request.shipment_type = msg->shipment_type;
  // m_get_shipment_priority_client.call(get_shipment_priority_srv); 
  // shipments_record[msg->shipment_type]->priority = get_shipment_priority_srv.response.priority; 

  // if (shipments_record[msg->shipment_type]->priority != 0) {
    // m_high_priorities_id.push_back(msg->shipment_type); 
  // }

}

bool AssemblyShipments::has_shipment() 
{
  return not m_new_shipments_id.empty(); 
}

void AssemblyShipments::update_part_task_queue(std::vector<std::tuple<int, std::unique_ptr<ariac_group1::PartTask>>>& part_task_queue)
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
        part_task.station_id = shipments_record[id]->shipment->station_id; 
        if (part_task.station_id == "as3" or
            part_task.station_id == "as4") {
            part_task.agv_id = "agv4"; 
        }
        else {
            part_task.agv_id = "agv2"; 
        }

        part_task.priority = shipments_record[id]->priority; 

        if (part_task.agv_id == "agv4") {
            part_task.priority++; 
        }

        // if (this->is_part_task_done(part_task)) {
        //     continue; 
        // }

        // if (part_task.priority != 0) {
        //     bool all_finish = true; 
        //     for (auto& high_priority_id: m_high_priorities_id) {
        //         if (high_priority_id == id) {
        //             continue; 
        //         }
        //         if (shipments_record[high_priority_id]->state != ShipmentState::FINISH) {
        //             all_finish = false; 
        //             break; 
        //         }
        //     }
        //
        //     if (all_finish) {
        //         // only set alert when new shipment is high priority and unfinished shipments are all low priority
        //         ROS_INFO("High priority alert!!!"); 
        //         m_high_priority_alert = true; 
        //     }
        // }
        part_task_queue.emplace_back(std::make_tuple(part_task.priority * Constants::PriorityWeight::Ratio::HIGH_PRIORITY,
                                                     std::make_unique<ariac_group1::PartTask>(part_task))); 
      }
    }

    m_new_shipments_id.clear(); 
  }
}

bool AssemblyShipments::is_high_priority_alert() 
{
    bool alert = m_high_priority_alert; 
    if (alert) {
      m_high_priority_alert = false; 
    }
    return alert; 
}

bool AssemblyShipments::is_part_task_done(const ariac_group1::PartTask& part_task) 
{
    ariac_group1::PartsUnderCamera srv; 
    srv.request.camera_id = "ks"; 
    srv.request.camera_id += part_task.agv_id.back(); 
    ROS_INFO("%s", srv.request.camera_id.c_str()); 
    m_parts_under_camera_client.call(srv); 

    auto target_pose_in_world = Utility::motioncontrol::transformToWorldFrame(
         part_task.part.pose,
         part_task.agv_id);

    nist_gear::Model target_part; 
    target_part.type = part_task.part.type; 
    target_part.pose = target_pose_in_world; 

    for (auto& part: srv.response.parts) {
      if (Utility::is_same_part(target_part, part, 0.05)) {
        ROS_INFO("agv has type: %s", part_task.part.type.c_str()); 
        shipments_record[part_task.shipment_type]->unfinished_part_tasks--; 
        if (shipments_record[part_task.shipment_type]->unfinished_part_tasks == 0) {
            // leave one last task for arm to submit shipment
            return false; 
        }
        return true; 
      }
    }

    return false; 
}

std::string AssemblyShipments::check_shipment_parts(ariac_group1::PartTask& part_task, nist_gear::Model& wrong_part)
{

    ariac_group1::PartsUnderCamera srv; 
    srv.request.camera_id = "ks"; 
    srv.request.camera_id += part_task.agv_id.back();
    m_parts_under_camera_client.call(srv); 

    // check for wrong type
    for (auto& product: shipments_record[part_task.shipment_type]->shipment->products) {
      auto target_pose_in_world = Utility::motioncontrol::transformToWorldFrame(
           product.pose,
           part_task.agv_id);

      nist_gear::Model target_part; 
      target_part.type = product.type; 
      target_part.pose = target_pose_in_world; 
      bool has_product = false; 
      for (auto& part: srv.response.parts) {
        if (Utility::is_same_part(target_part, part, 0.05)) {
          if (target_part.type != part.type) {
            part_task.part = product; 
            wrong_part = part;  
            return "wrong_type"; 
          }
          else {
            has_product = true; 
          }
          
          auto target_part_rpy = Utility::motioncontrol::eulerFromQuaternion(target_part.pose);
          auto part_rpy = Utility::motioncontrol::eulerFromQuaternion(part.pose);
          if (abs(target_part_rpy.at(2) - part_rpy.at(2)) > 0.1) {
            part_task.part = product; 
            wrong_part = part;  
            return "wrong_pose"; 
          }
        }
      }

      if (not has_product) {
        part_task.part = product; 
        return "missing_part"; 
      }
    }

    ROS_INFO("shipment_correct");  

    return "shipment_correct"; 

}

