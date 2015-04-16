#include <ros/ros.h>
#include <ros/package.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include "../lib/pcl/pc_aligner.h"

int main (int argc, char **argv)
{

	  // Load the target cloud PCD file
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::io::loadPCDFile (ros::package::getPath("dyn_3d_mod") + "/recordings/cloud1.pcd", *cloud1);
	  pcl::io::loadPCDFile (ros::package::getPath("dyn_3d_mod") + "/recordings/cloud2.pcd", *cloud2);

	  PCAligner aligner;
	  //target_cloud.setInputCloud (cloud1);
	  aligner.setInputClouds (cloud1, cloud2);


  return (0);
}
