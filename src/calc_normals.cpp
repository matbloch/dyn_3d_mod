#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <pcl/io/pcd_io.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

#include "../lib/pcl/pc_loader.h"

// features
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/shot.h>


// filtering
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>



int main(int argc, char** argv)
{


	/* ========================================== *\
	 * 		HELP
	\* ========================================== */

	if(argc<3){
		std::cout << "Please specify the running parameters" << std::endl;
		return -1;
	}

	if(argv[1] == std::string("help")){
		std::cout<<"\n";
		std::cout << "=================================" << std::endl;
		std::cout << " USAGE:" << std::endl;
		std::cout << "---------------------------------" << std::endl;
		std::cout << " --file (relative to /recordings)" << std::endl;
		std::cout << " --leaf_size (in meters)" << std::endl;
		std::cout << "=================================" << std::endl;
		std::cout<<"\n";
	}

	/* ========================================== *\
	 * 		LOAD CLOUD
	\* ========================================== */

	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(ros::package::getPath("dyn_3d_mod") + "/recordings/" + argv[1], *cloud) != 0)
	{
		return -1;
	}

	/* ========================================== *\
	 * 		DOWNSAMPLING
	\* ========================================== */

	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);

	float leaf_size;	// leaf size in m (only one point per {leaf_size} every cubic m will survive).
	leaf_size = std::atof(argv[2]);
	std::cout << "--- Setting leaf size to " << argv[2] << " meters" << std::endl;


	filter.setLeafSize(leaf_size,leaf_size,leaf_size);

	filter.filter(*filteredCloud);

	std::cout << "--- downsampling complete" << std::endl;

	/* ========================================== *\
	 * 		NORMAL CALUCLATION
	\* ========================================== */

	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);


	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(filteredCloud);
	normalEstimation.setRadiusSearch(1);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);


	// Visualize them.
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	viewer->addPointCloud<pcl::PointXYZ>(filteredCloud, "cloud");
	// Display one normal out of 20, as a line of length 3cm.
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(filteredCloud, normals, 1, 0.2, "normals");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;

}
