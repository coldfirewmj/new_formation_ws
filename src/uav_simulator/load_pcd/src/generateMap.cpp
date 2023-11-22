#include <iostream>
#include <vector>
#include <fstream>
#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
using namespace std;
int map_size_x;
int map_size_y;
int map_size_z;
double resolution;

void cw_rotate_90(vector<vector<int>> &matrix)
{
    int n = matrix.size();
    // Step 1: Transpose and reverse each row
    for (int i = 0; i < n; i++)
    {
        for (int j = i + 1; j < n; j++)
        {
            swap(matrix[i][j], matrix[j][i]);
        }
    }
    // Step 2: Reverse each row again
    for (int i = 0; i < n; i++)
    {
        reverse(matrix[i].begin(), matrix[i].end());
    }
}

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

vector<vector<int>> generate_map(pcl::PointCloud<pcl::PointXYZ>& cloud_)
{
    vector<vector<int>> int_map(map_size_x, vector<int>(map_size_y));
    //地面
    for (int i = -map_size_x/2; i < map_size_x/2+1; i++)
    {
        for (int j = -map_size_y/2; j < map_size_y/2+1; j++)
        {
            for (int k = 0; k < 1; k++)
            {
                double xd=i*resolution,yd=j*resolution,zd=k*resolution;
                cloud_.push_back(pcl::PointXYZ(xd, yd, zd));
            }
            /*if (i == 0 || j == 0 || i == map_size_x - 1 || j == map_size_y - 1)
            {
                int_map[i][j] = 1;
            }
            else
                int_map[i][j] = 0;*/
        }
    }
    //墙壁
    for (int i = -map_size_x/2; i < map_size_x / 2+1; i++)
    {
        int j = map_size_y / 2;
        for (int k = 0; k < map_size_z ; k++)
        {
            double xd=i*resolution,yd=j*resolution,zd=k*resolution;
            cloud_.push_back(pcl::PointXYZ(xd, yd, zd));
        }
        //int_map[i][j] = 1;
    }
    for (int i = -map_size_x/2; i < map_size_x / 2+1; i++)
    {
        int j = -map_size_y / 2;
        for (int k = 0; k < map_size_z ; k++)
        {
            double xd=i*resolution,yd=j*resolution,zd=k*resolution;
            cloud_.push_back(pcl::PointXYZ(xd, yd, zd));
        }
        //int_map[i][j] = 1;
    }
    for (int j = -map_size_y/2; j < map_size_y/2+1; j++)
    {
        int i = map_size_x / 2;
        for (int k = 0; k < map_size_z ; k++)
        {
            double xd=i*resolution,yd=j*resolution,zd=k*resolution;
            cloud_.push_back(pcl::PointXYZ(xd, yd, zd));
        }
        //int_map[i][j] = 1;
    }
    for (int j = -map_size_y/2; j < map_size_y/2+1; j++)
    {
        int i = -map_size_x / 2;
        for (int k = 0; k < map_size_z ; k++)
        {
            double xd=i*resolution,yd=j*resolution,zd=k*resolution;
            cloud_.push_back(pcl::PointXYZ(xd, yd, zd));
        }
        //int_map[i][j] = 1;
    }
    //内墙
    for (int i = -map_size_x/4; i < map_size_x / 4+1; i++)
    {
        int j = 0;
        for (int k = 0; k < map_size_z ; k++)
        {
            double xd=i*resolution,yd=j*resolution,zd=k*resolution;
            cloud_.push_back(pcl::PointXYZ(xd, yd, zd));
        }
        //int_map[i][j] = 1;
    }

    for (int i = 0; i < map_size_x / 2+1; i++)
    {
        int j = -map_size_y / 4;
        for (int k = 0; k < map_size_z ; k++)
        {
            double xd=i*resolution,yd=j*resolution,zd=k*resolution;
            cloud_.push_back(pcl::PointXYZ(xd, yd, zd));
        }
        //int_map[i][j] = 1;
    }

    for (int j = -map_size_y/ 2; j < map_size_y / 4+1; j++)
    {
        int i = -map_size_x / 4;
        for (int k = 0; k < map_size_z ; k++)
        {
            double xd=i*resolution,yd=j*resolution,zd=k*resolution;
            cloud_.push_back(pcl::PointXYZ(xd, yd, zd));
        }
        //int_map[i][j] = 1;
    }

    for (int j = map_size_y / 5; j < map_size_y / 2+1; j++)
    {
        int i = 0;
        for (int k = 0; k < map_size_z ; k++)
        {
            double xd=i*resolution,yd=j*resolution,zd=k*resolution;
            cloud_.push_back(pcl::PointXYZ(xd, yd, zd));
        }
        //int_map[i][j] = 1;
    }

    for (int j = 0; j < map_size_y / 4+1; j++)
    {
        int i = map_size_x / 4;
        for (int k = 0; k < map_size_z ; k++)
        {
            double xd=i*resolution,yd=j*resolution,zd=k*resolution;
            cloud_.push_back(pcl::PointXYZ(xd, yd, zd));
        }
        //int_map[i][j] = 1;
    }

    return int_map;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_generate");
    ros::NodeHandle nh("~");
    nh.param("map_size_x", map_size_x, 21);
    nh.param("map_size_y", map_size_y, 21);
    nh.param("map_size_z", map_size_z, 21);
    nh.param("resolution", resolution, 0.1);
    std::cout<<"map_size_x is:"<<map_size_x<<std::endl;
    pcl::PointCloud<pcl::PointXYZ> cloud_;
    sensor_msgs::PointCloud2 cloud_msg;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/pcd_topic", 1);
    std::ofstream fout("map_data.txt");
    std::vector<std::vector<int>> int_map = generate_map(cloud_);
    pcl::toROSMsg(cloud_, cloud_msg);
    ros::Time last_odom_stamp = ros::TIME_MAX;
    cloud_msg.header.frame_id = "world";
    cloud_msg.header.stamp    = last_odom_stamp;
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
    ROS_INFO("PCD file published!");
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
    pub.publish(cloud_msg);
    ros::spinOnce();
    loop_rate.sleep();
    }

    return 0;
}