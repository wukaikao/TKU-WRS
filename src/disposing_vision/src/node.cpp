#include <ros/ros.h>
#include <geometry_msgs/Point.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>

#include "pcl_tutorial.h"

ros::Publisher pub;
Pcl_tutorial Pcl_function;
float x_coordinate_min,y_coordinate_min,z_coordinate_min;
float x_coordinate_Max,y_coordinate_Max,z_coordinate_Max;
struct coordinate_normal
{
  int x,y,z;
  float normal_x,normal_y,normal_z;
}object_normal;

coordinate_normal average_normal(pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud){
  coordinate_normal average_normal;
  average_normal.normal_x = 0.0;
  average_normal.normal_y = 0.0;
  average_normal.normal_z = 0.0;
  int count = 0;
  for (size_t currentPoint = 0; currentPoint < input_cloud->points.size(); currentPoint++)
	{
    if(!(isnan(input_cloud->points[currentPoint].normal[0])||
         isnan(input_cloud->points[currentPoint].normal[1])||
         isnan(input_cloud->points[currentPoint].normal[2])))
    {
      average_normal.normal_x = average_normal.normal_x + input_cloud->points[currentPoint].normal[0];
      average_normal.normal_y = average_normal.normal_y + input_cloud->points[currentPoint].normal[1];
      average_normal.normal_z = average_normal.normal_z + input_cloud->points[currentPoint].normal[2];
		  count++;
      // std::cout << "Point:" << std::endl;
      // cout << currentPoint << std::endl;
		  // std::cout << "\town:" << average_normal.normal_x << " "
		  // 		                  << average_normal.normal_y << " "
		  // 		                  << average_normal.normal_z << std::endl;

		  // std::cout << "\tNormal:" << input_cloud->points[currentPoint].normal[0] << " "
		  // 		  << input_cloud->points[currentPoint].normal[1] << " "
		  // 		  << input_cloud->points[currentPoint].normal[2] << std::endl;
    }
  }
  printf("================\ncloud_size = %d \n",count);

  average_normal.normal_x = average_normal.normal_x/count;
  average_normal.normal_y = average_normal.normal_y/count;
  average_normal.normal_z = average_normal.normal_z/count;
  
  return average_normal;
}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // 将点云格式为sensor_msgs/PointCloud2 格式转为 pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg (*input, *cloud);   //关键的一句数据的转换
//------------------------------------------------------------------------------
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  //cylinder_segmentation
  // Pcl_function.cylinder_segmentation(cloud,object_cloud,plane_cloud);

  //passthrough
  // Pcl_function.passthrough(cloud,object_cloud,"x",x_coordinate_min,x_coordinate_Max);
  // Pcl_function.passthrough(cloud,object_cloud,"y",y_coordinate_min,y_coordinate_Max);
  Pcl_function.passthrough(cloud,object_cloud,"z",z_coordinate_min,z_coordinate_Max);

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal (new pcl::PointCloud<pcl::PointNormal>);
  Pcl_function.calculate_normal(object_cloud,cloud_normal);

  object_normal = average_normal(cloud_normal);
	std::cout << "\tTotal Normal:" << object_normal.normal_x << " "
			                           << object_normal.normal_y << " "
			                           << object_normal.normal_z << std::endl;
  
  //<<<Save file for debug>>>
  pcl::PCDWriter writer;
  writer.write ("object.pcd", *object_cloud, false);
  // writer.write ("plane.pcd", *plane_cloud, false);


  pcl::ModelCoefficients coefficients;
//------------------------------------------------------------------------------
  // 把提取出来的内点形成的平面模型的参数发布出去
  pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(coefficients, ros_coefficients);
  pub.publish (ros_coefficients);
}

void set_coordinate_limit_min(const geometry_msgs::Point& input)
{
  x_coordinate_min = input.x;
  printf("x_coordinatee_min = %f\n",x_coordinate_min);
  y_coordinate_min = input.y;
  printf("y_coordinatee_min = %f\n",y_coordinate_min);
  z_coordinate_min = input.z;
  printf("z_coordinatee_min = %f\n",z_coordinate_min);
}

void set_coordinate_limit_Max(const geometry_msgs::Point& input)
{
  x_coordinate_Max = input.x;
  printf("x_coordinate_Max = %f\n",x_coordinate_Max);  
  y_coordinate_Max = input.y;
  printf("y_coordinate_Max = %f\n",y_coordinate_Max);  
  z_coordinate_Max = input.z;
  printf("z_coordinate_Max = %f\n",z_coordinate_Max);  
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_node");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/cloud_pcd", 1, cloud_cb);
  ros::Subscriber sub_coordinate_min = nh.subscribe ("/coordinate_limit_min", 1, set_coordinate_limit_min);
  ros::Subscriber sub_coordinate_Max = nh.subscribe ("/coordinate_limit_Max", 1, set_coordinate_limit_Max);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);

  // Spin
  ros::spin ();
}