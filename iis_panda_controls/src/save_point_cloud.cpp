#include <ros/ros.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/common/io.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

void callback(const PointCloud::ConstPtr& msg)
{
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  
  pcl::io::savePCDFileASCII ("/home/marko/Desktop/IIS_Research/pcd_panda_view.pcd", *msg);
  std::cerr << "Saved " << (*msg).size() << " data points to test_pcd.pcd." << std::endl;

  ros::shutdown();
  // BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
  //   printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  
  ros::Subscriber sub = nh.subscribe<PointCloud>("/downsample/output", 10, callback);
  ROS_INFO("Subscribed waiting for topic...");
  ros::spin();
}