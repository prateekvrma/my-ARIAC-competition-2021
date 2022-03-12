#include <ros/ros.h>
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Model.h>
#include <string>

bool blackout = false;

void callback1(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    if(!msg->models.empty())
        ROS_INFO("Faulty part on agv 1...");

}
void callback2(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    if(!msg->models.empty())
        ROS_INFO("Faulty part on agv 2...");

}
void callback3(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    if(!msg->models.empty())
        ROS_INFO("Faulty part on agv 3...");

}
void callback4(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    if(!msg->models.empty())
        ROS_INFO("Faulty part on agv 4...");

}


int main(int argc, char **argv)
{
     ros::init(argc, argv, "qs"); 
     ros::Subscriber q1,q2,q3,q4;
     ros::NodeHandle nh;
     ros::Rate r(1000);
     q1 =  nh.subscribe("/ariac/quality_control_sensor_1", 1000, &callback1);
     q2 =  nh.subscribe("/ariac/quality_control_sensor_2", 1000, &callback2);
     q3 =  nh.subscribe("/ariac/quality_control_sensor_3", 1000, &callback3);
     q4 =  nh.subscribe("/ariac/quality_control_sensor_4", 1000, &callback4);
     while (ros::ok())
     {
         ros::spinOnce();
         r.sleep();
       
     }
}