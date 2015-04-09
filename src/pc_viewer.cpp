#include <ros/ros.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <pcl/io/pcd_io.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>


/* ========================================== *\
 * 		VIEWER
\* ========================================== */

int viewer(std::string filename)
{

	// specify file path
	std::string path = ros::package::getPath("dyn_3d_mod");
	path.append("/recordings/");
	path.append(filename); // load file

	// declare point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);

	// load file
	if(filename.substr(filename.find_last_of(".") + 1) == "pcd") {
		if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud1) != 0)
		{
			std::cout << "Could not load the point cloud." << std::endl;
			return -1;
		}
	} else if(filename.substr(filename.find_last_of(".") + 1) == "ply") {
		if (pcl::io::loadPLYFile(path, *cloud1) != 0)
		{
			std::cout << "Could not load the point cloud." << std::endl;
			return -1;
		}
	}else{
		std::cout << "Please use either .pcd or .ply files." << std::endl;
		return -1;
	}

	// visualize
	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	viewer.showCloud (cloud1);
	while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
	}

	return 0;

}

/* ========================================== *\
 * 		MAIN FUNCTION
\* ========================================== */


int main(int argc, char** argv)
{

	if(argc<2){
		std::cout << "Please specify the input file relative to the /storage direction." << std::endl;
	}

	viewer(argv[1]);

	return 0;

}
