#include <iostream>
#include <stdexcept>
#include <ros/package.h>

// PCL specific includes
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

/*
 * USAGE:
 * pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
 * cloud = load_pc(path);
 *
 */


pcl::PointCloud<pcl::PointXYZ>::Ptr load_pc(std::string path, bool package_relative = true)
{

	// declare point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	// specify file path
	if(package_relative){
		path = ros::package::getPath("dyn_3d_mod") + "/recordings/" + path;
	}

	// load file
	if(path.substr(path.find_last_of(".") + 1) == "pcd") {
		if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) != 0)
		{
			throw std::invalid_argument( "Could not load the point cloud." );
		}
	} else if(path.substr(path.find_last_of(".") + 1) == "ply") {
		if (pcl::io::loadPLYFile(path, *cloud) != 0)
		{
			throw std::invalid_argument( "Could not load the point cloud." );
		}
	}else{
		throw std::invalid_argument( "Please use either .pcd or .ply files." );
	}

	return cloud;

}
