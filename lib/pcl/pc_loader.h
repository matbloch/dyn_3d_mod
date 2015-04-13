#include <iostream>


// PCL specific includes
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>


pcl::PointCloud<pcl::PointXYZ> load_pc(std::string path)
{

	// declare point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	// load file
	if(path.substr(path.find_last_of(".") + 1) == "pcd") {
		if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) != 0)
		{
			std::cout << "Could not load the point cloud." << std::endl;
			return -1;
		}
	} else if(path.substr(path.find_last_of(".") + 1) == "ply") {
		if (pcl::io::loadPLYFile(path, *cloud) != 0)
		{
			std::cout << "Could not load the point cloud." << std::endl;
			return -1;
		}
	}else{
		std::cout << "Please use either .pcd or .ply files." << std::endl;
		return -1;
	}

	return *cloud;

}
