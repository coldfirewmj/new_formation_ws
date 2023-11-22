#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcd_reader");
  ros::NodeHandle nh("~");

  //读取PCD文件
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ> cloud_,cloud2;
  std::string package_path = ros::package::getPath("load_pcd");
  std::string pcd_file_relative_path = package_path + "/pcd_joint.pcd";
  //pcl::io::loadPCDFile ("/home/wmj/EGO-V2/EGO-Planner-v2-main/swarm-playground/main_ws/src/uav_simulator/load_pcd/AAA_process.pcd", cloud_);
  pcl::io::loadPCDFile (pcd_file_relative_path, cloud_);
  //pcl::io::loadPCDFile ("/home/wmj/EGO-Planner-v2/swarm-playground/main_ws/src/planner/load_pcd/vertices13.pcd", cloud2);
  /*if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/wmj/EGO-Planner-v2/swarm-playground/main_ws/src/planner/load_pcd/vertices.pcd", *cloud) == -1) 
  {
    PCL_ERROR ("Could not read PCD file! \n");
    return (-1);
  }*/
  for (int i=0;i<cloud_.size();i++)
  {
    
    pcl::PointXYZ pt=cloud_[i];
    auto x=pt.x;
    pt.x=pt.y;
    pt.y=x;
    //pt.z=-pt.z;
    pt.z+=2;
    cloud_[i]=pt;
  }

  //将点云转变为ROS格式
  sensor_msgs::PointCloud2 cloud_msg,cloud_msg2;
  pcl::toROSMsg(cloud_, cloud_msg);
  //pcl::toROSMsg(cloud2, cloud_msg2);

  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);
  //ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2>("pcd_topic2", 1);
  ros::Time last_odom_stamp = ros::TIME_MAX;
  cloud_msg.header.frame_id = "world";
  cloud_msg.header.stamp    = last_odom_stamp;

  cloud_msg2.header.frame_id = "world";
  cloud_msg2.header.stamp    = last_odom_stamp;
  ROS_INFO("PCD file published!");
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    pub.publish(cloud_msg);
    //pub2.publish(cloud_msg2);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

