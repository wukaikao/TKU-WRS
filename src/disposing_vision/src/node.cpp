#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>

#include "pcl_tutorial.h"

ros::Publisher pub;
Pcl_tutorial Pcl_function;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // 将点云格式为sensor_msgs/PointCloud2 格式转为 pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*input, *cloud);   //关键的一句数据的转换
//------------------------------------------------------------------------------
  pcl::PointCloud<pcl::PointXYZ> object_cloud,plane_cloud;

  Pcl_function.cylinder_segmentation(cloud,object_cloud,plane_cloud);

  //<<<Save file for debug>>>
  pcl::PCDWriter writer;
  writer.write ("object.pcd", object_cloud, false);
  writer.write ("plane.pcd", plane_cloud, false);


  pcl::ModelCoefficients coefficients;
//------------------------------------------------------------------------------
  // 把提取出来的内点形成的平面模型的参数发布出去
  pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(coefficients, ros_coefficients);
  pub.publish (ros_coefficients);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_node");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/cloud_pcd", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);
  
  // Spin
  ros::spin ();
}