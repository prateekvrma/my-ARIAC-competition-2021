#include <ros/ros.h>
#include <nist_gear/LogicalCameraImage.h>

bool blackout = false;

void callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // ROS_INFO("Sensors are Running...");
    blackout = false;

}

int main(int argc, char **argv)
{
     ros::init(argc, argv, "test_bs"); 
     ros::Subscriber s_b;
     ros::NodeHandle nh;
     ros::Rate r(10);
     s_b =  nh.subscribe("/ariac/logical_camera_bins0", 1000, &callback);
     while (ros::ok())
     {
         blackout = true;
         ros::spinOnce();
         r.sleep();
         if(blackout == true)
         {
             ROS_INFO("Blackout...");
         }
     }
}