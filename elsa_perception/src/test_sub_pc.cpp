#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/io.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

PointCloud PC;

pcl::visualization::PCLVisualizer::Ptr normalsVis (
     pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
 {
   // --------------------------------------------------------
   // -----Open 3D viewer and add point cloud and normals-----
   // --------------------------------------------------------
   pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
   viewer->setBackgroundColor (0, 0, 0);
   pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
   viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
   viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
   viewer->addCoordinateSystem (1.0);
   viewer->initCameraParameters ();
   return (viewer);
 }

void callback(const PointCloud::ConstPtr& msg)
{
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  
  // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  // viewer.showCloud (msg);
  // while (!viewer.wasStopped ())
  //   {
  //   }

  // Create the normal estimation class, and pass the input dataset to it
  // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

  // pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_pc (new pcl::PointCloud<pcl::PointXYZ>);

  // pcl::copyPointCloud(msg, &xyz_pc);

  // ne.setInputCloud (xyz_pc);
  // // Create an empty kdtree representation, and pass it to the normal estimation object.
  // // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  // ne.setSearchMethod (tree);

  // // Output datasets
  // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  // // Use all neighbors in a sphere of radius 3cm
  // ne.setRadiusSearch (0.005);
  // // Compute the features
  // ne.compute (*cloud_normals);

  // normalsVis(msg, cloud_normals);
  ros::shutdown();
  // pcl::visualization::PCLVisualizer::Ptr viewer;
  // viewer = normalsVis(msg, cloud_normals);
  // while (!(viewer)->wasStopped())
  // {
  // }
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