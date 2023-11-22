#include <ros/ros.h>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <nav_msgs/Odometry.h>
boost::shared_ptr<nav_msgs::Odometry const> msg;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    //ros::Subscriber presub = nh.subscribe<nav_msgs::Odometry>("/drone_1_visual_slam/odom", 1, [](nav_msgs::Odometry::ConstPtr) {});
    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce(); 
        ros::Time T1=ros::Time::now();
        ros::topic::waitForMessage<nav_msgs::Odometry>("/drone_1_visual_slam/odom",nh);
        ros::Time T2=ros::Time::now();
        std::cout<<"time is: "<<(T2-T1).toSec()*1000<<"ms"<<std::endl;
        rate.sleep();
    }
    
    return 0;
}