#include "station.h"

#include <nist_gear/AssemblyStationSubmitShipment.h>
#include <ariac_group1/Busy.h>

using AssemSubmit = nist_gear::AssemblyStationSubmitShipment; 

Station::Station(ros::NodeHandle* nodehandle, const std::string& id):
  m_nh{*nodehandle}, 
  m_id{id}
{  
  m_submit_shipment_client = 
      m_nh.serviceClient<AssemSubmit>("/ariac/" + m_id + "/submit_shipment"); 
  m_submit_shipment_client.waitForExistence();
}

void Station::submit_shipment(const std::string& shipment_type)
{  
  AssemSubmit srv; 
  srv.request.shipment_type = shipment_type; 

  if (m_submit_shipment_client.call(srv)) {
    ROS_INFO("Calling service submit_shipment: %s", shipment_type.c_str()); 
    ROS_INFO_STREAM("inspection result: " << srv.response.inspection_result); 
  }
  else{
    ROS_ERROR("Failed to submit shipment: %s", shipment_type.c_str()); 
  }

}
