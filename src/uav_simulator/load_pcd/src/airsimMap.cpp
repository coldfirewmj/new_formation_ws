#include <iostream>
#include <vector>
#include <fstream>
#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>

using namespace std;
int map_size_x;
int map_size_y;
int map_size_z;
double resolution;
vector<Eigen::Vector2i> obstacle_list;

// 障碍物膨胀函数
void dilate(std::vector<std::vector<int>> &img, int iterations)
{
    int h = img.size();
    int w = img[0].size();
    std::vector<std::vector<int>> new_img(h, std::vector<int>(w, 0));

    // 多次膨胀
    for (int k = 0; k < iterations; k++)
    {
        // 遍历每个像素点
        for (int i = 0; i < h; i++)
        {
            for (int j = 0; j < w; j++)
            {
                // 如果当前像素为1，则将周围的0变成1
                if (img[i][j] == 1)
                {
                    if (i > 0 && img[i - 1][j] == 0)
                        new_img[i - 1][j] = 1; // 上
                    if (j > 0 && img[i][j - 1] == 0)
                        new_img[i][j - 1] = 1; // 左
                    if (i < h - 1 && img[i + 1][j] == 0)
                        new_img[i + 1][j] = 1; // 下
                    if (j < w - 1 && img[i][j + 1] == 0)
                        new_img[i][j + 1] = 1; // 右
                    new_img[i][j] = 1;         // 当前像素点
                }
            }
        }

        // 将新图像赋值给原图像，为下一次膨胀做准备
        img = new_img;
    }
}

void generate_map(pcl::PointCloud<pcl::PointXYZ>& cloud_)
{
    Eigen::Vector2i obp;
    //地面
    /*for (float i = -15.3; i <= 686.8; i++)
    {
        for (float j = -351.4; j <= 350.7; j++)
        {
            for (float k = 0; k < 1; k++)
            {
                double xd=i*resolution,yd=j*resolution,zd=k*resolution;
                cloud_.push_back(pcl::PointXYZ(xd, yd, zd));
            }
        }
    }*/
    //建筑
    for(float startx=14.6; startx<656.7;startx+=112)
    {
        for(float starty=-321.4; starty<320.7; starty+=112)
        {
            for(float i = startx;i <= startx+82.1; i++)
            {
                for (float j = starty; j <= starty+82.1 ; j++)
                {
                    obp(0)=i;
                    obp(1)=j;
                    obstacle_list.push_back(obp);
                    if (i>=startx+5&&i<startx+77.1&&j>=starty+5&&j<starty+77.1)
                    {
                        double xd=i*resolution,yd=j*resolution,zd=50*resolution;
                        cloud_.push_back(pcl::PointXYZ(xd, yd, zd));
                        continue;
                    }
                    for (float k = 0; k < 50 ; k++)
                    {
                        double xd=i*resolution,yd=j*resolution,zd=k*resolution;
                        cloud_.push_back(pcl::PointXYZ(xd, yd, zd));
                    }
                }
            }
        }
    }
    //障碍1
    for(float i = 14.6+112;i <= 14.6+112+82.1; i++)
    {
        for (float j = -321.4+112*2+82.1; j <= -321.4+112*3 ; j++)
        {
            obp(0)=i;
            obp(1)=j;
            obstacle_list.push_back(obp);
            for (float k = 0; k < 50 ; k++)
            {
                double xd=i*resolution,yd=j*resolution,zd=k*resolution;
                cloud_.push_back(pcl::PointXYZ(xd, yd, zd));
            }
        }
    }
    //障碍2
    for(float i = 14.6+112+82.1;i <= 14.6+112+112; i++)
    {
        for (float j = -321.4+112; j <= -321.4+112+82.1 ; j++)
        {
            obp(0)=i;
            obp(1)=j;
            obstacle_list.push_back(obp);
            for (float k = 0; k < 50 ; k++)
            {
                double xd=i*resolution,yd=j*resolution,zd=k*resolution;
                cloud_.push_back(pcl::PointXYZ(xd, yd, zd));
            }
        }
    }
    //障碍3
    for(float i = 14.6+112*3+82.1;i <= 14.6+112*4; i++)
    {
        for (float j = -321.4+112*2; j <= -321.4+112*2+82.1 ; j++)
        {
            obp(0)=i;
            obp(1)=j;
            obstacle_list.push_back(obp);
            for (float k = 0; k < 50 ; k++)
            {
                double xd=i*resolution,yd=j*resolution,zd=k*resolution;
                cloud_.push_back(pcl::PointXYZ(xd, yd, zd));
            }
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_generate");
    ros::NodeHandle nh("~");
    nh.param("map_size_x", map_size_x, -4183);
    nh.param("map_size_y", map_size_y, 33628);
    nh.param("map_size_z", map_size_z, 21);
    nh.param("resolution", resolution, 0.1);
    pcl::PointCloud<pcl::PointXYZ> cloud_,true_cloud_;
    sensor_msgs::PointCloud2 cloud_msg,true_cloud_msg;

    visualization_msgs::Marker wayPointsMarker;

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);
    ros::Publisher true_pub=nh.advertise<sensor_msgs::PointCloud2>("/true_pcd_topic", 1);
    // std::ofstream fout("map_data.txt");
    generate_map(cloud_);
    pcl::toROSMsg(cloud_, cloud_msg);
    /*true_cloud_=cloud_;
    for (unsigned i=0;i<true_cloud_.points.size();i++)
    {
        pcl::PointXYZ pt=true_cloud_.points[i];
        pt.x=pt.x*0.1;
        pt.y=pt.y*0.1;
        pt.z=pt.z*0.1;
        true_cloud_.points[i]=pt;
    }
    pcl::toROSMsg(true_cloud_, true_cloud_msg);*/
    ros::Time last_odom_stamp = ros::TIME_MAX;
    cloud_msg.header.frame_id = "world";
    cloud_msg.header.stamp    = last_odom_stamp;

    true_cloud_msg.header.frame_id = "world";
    true_cloud_msg.header.stamp    = last_odom_stamp;
    /*cw_rotate_90(int_map);

    for (auto vec : int_map)
    {
        for (auto val : vec)
        {
            cout << val << ' ';
        }
        cout << '\n';
    }
    std::cout << std::endl;

    dilate(int_map, 0);

    for (auto vec : int_map)
    {
        for (auto val : vec)
        {
            cout << val << ' ';
        }
        cout << '\n';
    }

    std::cout << std::endl;
    for (auto vec : int_map)
    {
        for (auto val : vec)
        {
            fout << val << ' ';
        }
        fout << '\n';
    }
    fout.close();*/
    ROS_INFO("airsim map published!");
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
    pub.publish(cloud_msg);
    true_pub.publish(true_cloud_msg);
    ros::spinOnce();
    loop_rate.sleep();
    }

    return 0;
}