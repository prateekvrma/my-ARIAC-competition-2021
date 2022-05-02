#include "sensor_manager.h"

#include <tuple>

SensorManager::SensorManager(ros::NodeHandle* nodehandle):
  m_nh{*nodehandle}
{
  m_get_parts_service = m_nh.advertiseService("/sensor_manager/get_parts", &SensorManager::get_parts, this); 
  m_is_faulty_service = m_nh.advertiseService("/sensor_manager/is_faulty", &SensorManager::is_faulty, this); 
  m_is_shipment_ready_service = m_nh.advertiseService("/sensor_manager/is_shipment_ready", &SensorManager::is_shipment_ready, this); 
  m_parts_in_camera_service = m_nh.advertiseService("/sensor_manager/parts_in_camera", &SensorManager::parts_in_camera, this); 
  m_is_part_picked_service = m_nh.advertiseService("/sensor_manager/is_part_picked", &SensorManager::is_part_picked, this); 
  m_get_part_position_service = m_nh.advertiseService("/sensor_manager/get_part_position", &SensorManager::get_part_position, this); 
  m_check_quality_sensor_service = m_nh.advertiseService("/sensor_manager/check_quality_sensor", &SensorManager::check_quality_sensor, this); 
  m_get_vacancy_pose_service = m_nh.advertiseService("/sensor_manager/get_vacancy_pose", &SensorManager::get_vacancy_pose, this); 
  m_is_belt_sensor_triggered_service = m_nh.advertiseService("/sensor_manager/is_belt_sensor_triggered", &SensorManager::is_belt_sensor_triggered, this); 
  m_get_belt_part_service = m_nh.advertiseService("/sensor_manager/get_belt_part", &SensorManager::get_belt_part, this); 
  m_get_belt_proximity_sensor_service = m_nh.advertiseService("/sensor_manager/get_belt_proximity_sensor", &SensorManager::get_belt_proximity_sensor, this); 
  m_parts_under_camera_service = m_nh.advertiseService("/sensor_manager/parts_under_camera", &SensorManager::parts_under_camera, this); 

  // All Logical cameras in the environment
  for (auto& camera_id: m_logical_cameras) {
    std::string prefix = "logical_camera_"; 
    m_logical_cameras_dict[camera_id] = std::make_unique<LogicalCamera>(nodehandle, prefix += camera_id); 
  }

  // All quality sensors in the environment
  for (auto& camera_id: m_quality_sensors) {
    m_quality_sensors_dict[camera_id] = std::make_unique<LogicalCamera>(nodehandle, camera_id); 
  }

  m_belt_breakbeam = std::make_unique<BreakBeam>(nodehandle, m_belt_breakbeam_id);
  m_belt_camera = std::make_unique<LogicalCamera>(nodehandle, m_belt_camera_id); 
  m_belt_proximity_sensor = std::make_unique<ProximitySensor>(nodehandle, m_belt_proximity_sensor_id); 

  for (auto& bin_id: m_bins_id) {
    bins_occupancy[bin_id].resize(4); 
    for (int i=0; i<4; i++) {
      bins_occupancy[bin_id].at(i) = false; 
    }
  }

}

void SensorManager::update_parts()
{
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 

  for (auto& bin_id: m_bins_id) {
    for (int i=0; i<4; i++) {
      bins_occupancy[bin_id].at(i) = false; 
    }
  }

  for (auto& data: m_parts_database) {
    auto key = data.first; 
    for (auto& part_info_ptr: data.second) {
      if (part_info_ptr  == nullptr) {
          continue; 
      }
      auto loc = Utility::location::get_pose_location(part_info_ptr->part.pose); 
      for (auto& bin_id: m_bins_id) {
          if (bin_id == loc) {
              auto loc_in_bin = Utility::location::get_pose_location_in_bin(part_info_ptr->part.pose, bin_id); 
              bins_occupancy[bin_id].at(loc_in_bin) = true; 
              break; 
          }
      }
    }
  }

  // this->print_bins_occupancy(); 

  if (m_sensors_blackout) {
      return; 
  }

  m_parts_database.clear(); 

  for (auto& camera_id: m_logical_cameras) {
    m_logical_cameras_dict[camera_id]->update_parts(m_parts_database);  
  }
  
  for (auto& camera_id: m_quality_sensors) {
    m_quality_sensors_dict[camera_id]->update_parts(m_parts_database);  
  }

  
}

void SensorManager::print_bins_occupancy()
{
  for (auto& bin_id: m_bins_id) {
    ROS_INFO("%s: ", bin_id.c_str()); 
    std::string pos_1 = " "; 
    std::string pos_2 = " "; 
    std::string pos_3 = " "; 
    std::string pos_4 = " "; 
    if (bins_occupancy[bin_id].at(0)) {
      pos_1 = "O"; 
    }
    if (bins_occupancy[bin_id].at(1)) {
      pos_2 = "O"; 
    }
    if (bins_occupancy[bin_id].at(2)) {
      pos_3 = "O"; 
    }
    if (bins_occupancy[bin_id].at(3)) {
      pos_4 = "O"; 
    }
    ROS_INFO("  %s  %s", pos_1.c_str(), pos_2.c_str()); 
    ROS_INFO("  %s  %s", pos_3.c_str(), pos_4.c_str()); 
  }
}

void SensorManager::show_database()
{
  for (auto& data: m_parts_database) {
    auto key = data.first; 
    ROS_INFO("Key: %s", key.c_str()); 
    for (auto& part_info_ptr: data.second) {
      Utility::print_part_pose(part_info_ptr->part); 
    }
  }
}

void SensorManager::check_blackout() 
{
  static int test_count = 0; 

  for (auto& camera_id: m_logical_cameras) {
    if (not m_logical_cameras_dict[camera_id]->is_blackout()) {
      // If one of the sensors report no blackout, then no blackout
      // set test_blackout for next time
      m_logical_cameras_dict[camera_id]->reset_blackout(); 
      m_sensors_blackout = false; 
      test_count = 0; 
      return; 
    }
  }

  if (test_count < 5) {

    test_count++; 

  } 
  else {

    ROS_INFO("Sensors blackout"); 
    m_sensors_blackout = true; 
    test_count = 0; 
  }

}

bool SensorManager::get_parts(ariac_group1::GetParts::Request &req, 
                              ariac_group1::GetParts::Response &res) 
{
  ros::spinOnce(); 

  std::vector<std::string> kitting_arm_areas = {"bin1", "bin2", "bin5", "bin6",
                                                "ks1", "ks2", "ks3", "ks4"}; 
   
  for (auto& part_info_ptr: m_parts_database[req.type]) {
    if (part_info_ptr == nullptr) {
        continue; 
    }
    auto part_loc = Utility::location::get_pose_location(part_info_ptr->part.pose); 
    for (auto& loc: kitting_arm_areas) {
        if (part_loc == loc) {
            res.parts_info.push_back(*part_info_ptr); 
            Utility::print_part_pose(part_info_ptr->part); 
            break; 
        }
    }
  }

  return true; 
}

bool SensorManager::is_faulty(ariac_group1::IsFaulty::Request &req, 
                              ariac_group1::IsFaulty::Response &res) 
{
  if (m_sensors_blackout) {
    ROS_INFO("Sensor blackout: Assume part not faulty"); 
    res.faulty = false; 
    return true;  
  }

  for (auto& part_info_ptr: m_parts_database["model"]) {
    Utility::print_part_pose(part_info_ptr->part); 
    if (Utility::is_same_part(part_info_ptr->part, req.part, 0.1)) {
      res.faulty = true; 
      return true; 
    }
  }

  res.faulty = false; 

  return true; 
}

bool SensorManager::is_shipment_ready(ariac_group1::IsShipmentReady::Request &req, 
                                      ariac_group1::IsShipmentReady::Response &res) 
{
  auto agv_id = req.shipment.agv_id; 
  std::string camera_id; 
  if (agv_id == "agv1") {
    camera_id = "ks1"; 
  }
  if (agv_id == "agv2") {
    camera_id = "ks2"; 
  }
  if (agv_id == "agv3") {
    camera_id = "ks3"; 
  }
  if (agv_id == "agv4") {
    camera_id = "ks4"; 
  }

  bool ready = true; 
  for (auto& product: req.shipment.products) {
    nist_gear::Model order_part; 
    order_part.type = product.type; 
    order_part.pose = product.pose; 

    bool same_part = false; 
    for (auto& part_info_ptr: m_parts_database[product.type]) {
      if (Utility::is_same_part(order_part, part_info_ptr->part, 0.2)) {
        same_part = true; 
        break; 
      }
    }
    if (not same_part) {
      ready = false; 
      break; 
    }
  }
  res.ready = ready; 
  return true; 
}

bool SensorManager::parts_in_camera(ariac_group1::PartsInCamera::Request &req, 
                                    ariac_group1::PartsInCamera::Response &res) 
{
  ros::spinOnce(); 
  std::string id = this->convert_id_to_internal(req.camera_id); 
  if (m_logical_cameras_dict.count(id)) {
    res.parts_amount = m_logical_cameras_dict[id]->parts_world_frame.size(); 
    return true; 
  }

  if (m_quality_sensors_dict.count(id)) {
    res.parts_amount = m_quality_sensors_dict[id]->parts_world_frame.size(); 
    return true; 
  }

  return false; 
}

std::string SensorManager::convert_id_to_internal(std::string global_id)
{
  std::string internal_id; 
  if (global_id.compare("logical_camera_bins0") == 0) {
    internal_id = "bins0"; 
  }
  else if (global_id.compare("logical_camera_bins1") == 0) {
    internal_id = "bins1"; 
  }
  else if (global_id.compare("logical_camera_ks1") == 0) {
    internal_id = "ks1"; 
  }
  else if (global_id.compare("logical_camera_ks2") == 0) {
    internal_id = "ks2"; 
  }
  else if (global_id.compare("logical_camera_ks3") == 0) {
    internal_id = "ks3"; 
  }
  else if (global_id.compare("logical_camera_ks4") == 0) {
    internal_id = "ks4"; 
  }
  else if (global_id.compare("logical_camera_bfc1") == 0) {
    internal_id = "bfc1"; 
  }
  else if (global_id.compare("logical_camera_bfc2") == 0) {
    internal_id = "bfc2"; 
  }
  else if (global_id.compare("logical_camera_bfc3") == 0) {
    internal_id = "bfc3"; 
  }
  else if (global_id.compare("logical_camera_bfc4") == 0) {
    internal_id = "bfc4"; 
  }

  if (global_id.find("quality_control_sensor") != std::string::npos) {
    internal_id = global_id; 
  }

  return internal_id; 
}

bool SensorManager::is_part_picked(ariac_group1::IsPartPicked::Request &req,
                                   ariac_group1::IsPartPicked::Response &res) 
{
  if (m_sensors_blackout) {
    ROS_INFO("Sensor blackout, assuming pick success"); 
    res.picked = true; 
    // clear from database
    for (auto& part_info_ptr: m_parts_database[req.part.type]) {
      if (Utility::is_same_part(req.part, part_info_ptr->part, 0.05)) {
        ROS_INFO("Clear part at x: %f, y: %f", req.part.pose.position.x, req.part.pose.position.y); 
        part_info_ptr.reset(nullptr); 
        // erase null pointer in database
        m_parts_database[req.part.type].erase(std::remove(m_parts_database[req.part.type].begin(),
                                                          m_parts_database[req.part.type].end(),
                                                          nullptr)); 
        ROS_INFO("Part %s left in database: %d", req.part.type.c_str(), int(m_parts_database[req.part.type].size())); 
        break; 
      }
    }
    return true;  
  }

  std::string id = this->convert_id_to_internal(req.camera_id); 
  double platform_height = -1; 
  if (id.find("ks") != std::string::npos) {
    platform_height = 0.76;  
  }
  else if (id.find("bins") != std::string::npos) {
    platform_height = 0.76;  
  }
  
  ros::spinOnce(); 
  res.picked = false;  
  if (m_logical_cameras_dict.count(id)) {
    for (auto &part: m_logical_cameras_dict[id]->parts_world_frame) {
      if (part == nullptr) {
        continue; 
      }
      if (part->type != req.part.type) {
        continue; 
      }
      double picked_margin = 0.03; 
      ROS_INFO("Is part picked height: %f", part->pose.position.z); 
      if ((part->pose.position.z - platform_height) > picked_margin) {
        res.picked = true; 

        // clear from database
        for (auto& part_info_ptr: m_parts_database[part->type]) {
          if (Utility::is_same_part(*part, part_info_ptr->part, 0.05)) {
            ROS_INFO("Clear part at x: %f, y: %f", part->pose.position.x, part->pose.position.y); 
            part_info_ptr.reset(nullptr); 
            // erase null pointer in database
            m_parts_database[part->type].erase(std::remove(m_parts_database[part->type].begin(),
                                                           m_parts_database[part->type].end(),
                                                           nullptr)); 

            ROS_INFO("Part %s left in database: %d", part->type.c_str(), int(m_parts_database[part->type].size())); 
            break; 
          }
        }
        part.reset(nullptr); 

        return true; 
      }
    }
    ROS_INFO("No part get picked"); 
    res.picked = false; 
    return true; 
  }
  else {
    ROS_INFO("No camera: %s", req.camera_id.c_str()); 
    return false; 
  }

}

bool SensorManager::get_part_position(ariac_group1::GetPartPosition::Request &req,
                                      ariac_group1::GetPartPosition::Response &res)
{
  std::string id = this->convert_id_to_internal(req.camera_id); 
  for (auto &part_ptr: m_logical_cameras_dict[id]->parts_world_frame) {
    if (part_ptr == nullptr) {
        continue; 
    }
    if (Utility::is_same_part(*part_ptr, req.part, 0.1)) {
      ROS_INFO("Part position correction"); 
      res.pose = part_ptr->pose; 
      Utility::print_part_pose(*part_ptr); 
      return true; 
    }
  }

  return false; 
}

bool SensorManager::check_quality_sensor(ariac_group1::CheckQualitySensor::Request &req,
                                         ariac_group1::CheckQualitySensor::Response &res)
{
  if (m_sensors_blackout) {
    ROS_INFO("Sensor blackout: Cannot check quality sensor"); 
    return false;  
  }

  std::string global_quality_sensor_id; 
  std::string global_logical_camera_id; 

  if (req.agv_id.find("agv1") != std::string::npos) {
    global_quality_sensor_id = "quality_control_sensor_1";
    global_logical_camera_id = "logical_camera_ks1"; 
  }
  else if (req.agv_id.find("agv2") != std::string::npos) {
    global_quality_sensor_id = "quality_control_sensor_2";
    global_logical_camera_id = "logical_camera_ks2"; 
  }
  else if (req.agv_id.find("agv3") != std::string::npos) {
    global_quality_sensor_id = "quality_control_sensor_3";
    global_logical_camera_id = "logical_camera_ks3"; 
  }
  else if (req.agv_id.find("agv4") != std::string::npos) {
    global_quality_sensor_id = "quality_control_sensor_4";
    global_logical_camera_id = "logical_camera_ks4"; 
  }

  res.camera_id = global_logical_camera_id; 
  std::string quality_sensor_id = this->convert_id_to_internal(global_quality_sensor_id); 
  std::string logical_camera_id = this->convert_id_to_internal(global_logical_camera_id); 


  for (auto &logical_part_ptr: m_logical_cameras_dict[logical_camera_id]->parts_world_frame) {
    if (logical_part_ptr == nullptr) {
        continue; 
    }

    for (auto &quality_part_ptr: m_quality_sensors_dict[quality_sensor_id]->parts_world_frame) {

      if (Utility::is_same_part(*logical_part_ptr, *quality_part_ptr, 0.05)) {

        nist_gear::Model faulty_part; 
        faulty_part.type = logical_part_ptr->type; 
        faulty_part.pose = logical_part_ptr->pose; 
        res.faulty_parts.push_back(faulty_part); 

      }

    }

  }
  return true; 
}

bool SensorManager::get_vacancy_pose(ariac_group1::GetVacancyPose::Request &req,
                                     ariac_group1::GetVacancyPose::Response &res)
{
  std::vector<std::string> test_bins{"bin1", "bin2", "bin5", "bin6"}; 
  for (auto& bin_id: test_bins) {
    bins_occupancy[bin_id].resize(4); 
    for (int i=0; i<4; i++) {
      if (not bins_occupancy[bin_id].at(i)) {
        auto pose = Utility::location::get_pose_from_bin_location(bin_id, i); 
        res.vacancy_poses.push_back(pose); 
        bins_occupancy[bin_id].at(i) = true; 
        return true; 
      }
    }
  }
  return true; 
}

bool SensorManager::is_belt_sensor_triggered(std_srvs::Trigger::Request &req,
                                             std_srvs::Trigger::Response &res)
{
    res.success = false; 
    if (m_sensors_blackout) {
        // assume not triggered
        return true; 
    }

    if (m_belt_breakbeam->is_triggered()) {
        res.success = true; 
        m_belt_breakbeam->reset_triggered(); 
    }

    return true; 
}

bool SensorManager::get_belt_part(ariac_group1::GetBeltPart::Request &req,
                                  ariac_group1::GetBeltPart::Response &res)
{
  ros::spinOnce(); 

  if (m_sensors_blackout) {
      // assume no part
      return true; 
  }

  m_belt_camera->update_parts(); 
  for (auto& part_ptr: m_belt_camera->parts_world_frame) {
      if (part_ptr == nullptr) {
          continue; 
      }
      res.part = *part_ptr; 
      break; 
  }

  return true; 

}

bool SensorManager::get_belt_proximity_sensor(ariac_group1::GetBeltProximitySensor::Request &req,
                                              ariac_group1::GetBeltProximitySensor::Response &res)
{
  ros::spinOnce(); 

  if (m_sensors_blackout) {
      // assume no part
      return true; 
  }

  res.range = m_belt_proximity_sensor->get_object_range(); 
  m_belt_proximity_sensor->reset_object_range(); 

  return true; 
}

bool SensorManager::parts_under_camera(ariac_group1::PartsUnderCamera::Request &req,
                                       ariac_group1::PartsUnderCamera::Response &res)
{
    if (m_sensors_blackout) {
      // assume no part
      return false; 
    }

    ros::spinOnce(); 
    if (req.camera_id.find("belt") != std::string::npos) {
       m_belt_camera->update_parts(); 
       for (auto& part_ptr: m_belt_camera->parts_world_frame) {
         if (part_ptr == nullptr) {
            continue; 
         }
         res.parts.push_back(*part_ptr); 
       }
       return true; 
    }

    if (m_logical_cameras_dict.count(req.camera_id)) {
      for (auto& part_ptr: m_logical_cameras_dict[req.camera_id]->parts_world_frame) {
          if (part_ptr == nullptr) {
              continue; 
          }
          res.parts.push_back(*part_ptr); 
      }
      
      return true; 
    }
    else {
      ROS_INFO("No camera: %s", req.camera_id.c_str()); 
      return false; 
    }
}
