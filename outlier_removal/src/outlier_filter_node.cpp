#include<ros/ros.h> //ros标准库头文件  
#include<iostream> //C++标准输入输出库  
#include <stdio.h>  
/*  
  以下是pointcloud2转化的必要的头文件  
*/  
#include <sensor_msgs/PointCloud2.h>  
#include <pcl/point_types.h>  
#include <pcl/filters/voxel_grid.h>  
#include <pcl_conversions/pcl_conversions.h>  
#include <pcl/point_cloud.h>  
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
  
static const std::string pointstopic = "/voxel_filter";//设置订阅topic名称  
  
ros::Publisher points_pub;//定义ROS消息发布器  
/*以下为滤波函数，首先需要将sensor_msg::PointCloud2格式转化成pcl::PCLPointCloud2格式，然后再使用VoxelGrid滤波 */  
void pointsdo(const sensor_msgs::PointCloud2ConstPtr& msg)      
  { 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    //////
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 msg_filtered;
    pcl_conversions::toPCL(*msg, *cloud);       //sensor_msgs::PointCloud2ConstPtr -> pcl::PCLPointCloud2

    sensor_msgs::PointCloud2 point_output;

    // 创建滤波器
    pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> outrem;
    outrem.setInputCloud(cloudPtr);
    outrem.setRadiusSearch(1.0);
    outrem.setMinNeighborsInRadius (2);
    // 应用滤波器
    outrem.filter (msg_filtered);
    pcl_conversions::fromPCL(msg_filtered, point_output);
    points_pub.publish(point_output);
  }  
  
int main(int argc, char** argv)  
   {  
     ros::init(argc, argv, "filter_node");  
     ros::NodeHandle nh;  
     ros::Subscriber sub = nh.subscribe (pointstopic, 1, pointsdo);  
     points_pub=nh.advertise<sensor_msgs::PointCloud2>("/outlier_filter", 1);  
     ros::spin();  
     return 0;  
   }  
