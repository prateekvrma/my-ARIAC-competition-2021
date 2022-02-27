#include "station.h"

#include <nist_gear/AssemblyStationSubmitShipment.h>

using AssemSubmit = nist_gear::AssemblyStationSubmitShipment; 

Station::Station(ros::NodeHandle* nodehandle, const std::string &id):
  m_nh{*nodehandle}, 
  m_id{id}{
}

void Station::submit_shipment(const std::string &shipment_type){  

  auto service_name = "/ariac/" + m_id + "/submit_shipment"; 
  static auto client = m_nh.serviceClient<AssemSubmit>(service_name); 

  if (!client.exists())
  {
    ROS_INFO("Waiting for the competition to be ready...");
    client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }

  AssemSubmit srv; 
  srv.request.shipment_type = shipment_type; 

  if (client.call(srv)){
    ROS_INFO_STREAM("inspection result: " << srv.response.inspection_result); 
  }
  else{
    ROS_ERROR("Failed to call %s", service_name.c_str()); 
  }

}
