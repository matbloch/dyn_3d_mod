#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <pcl/io/pcd_io.h>

#include <string>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>


#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>


#include <Eigen/Geometry>

#include <array>
#include <limits>

/* ========================================== *\
 * 		EULER ANGLES
\* ========================================== */


typedef std::array<float, 3> float3;
typedef std::array<float3, 3> float3x3;

const float PI = 3.14159265358979323846264f;

bool closeEnough(const float& a, const float& b, const float& epsilon = std::numeric_limits<float>::epsilon()) {
    return (epsilon > std::abs(a - b));
}

float3 eulerAngles(const float3x3& R) {

    //check for gimbal lock
    if (closeEnough(R[0][2], -1.0f)) {
        float x = 0; //gimbal lock
        float y = PI / 2;
        float z = x + atan2(R[1][0], R[2][0]);
        return float3 { x, y, z };
    } else if (closeEnough(R[0][2], 1.0f)) {
        float x = 0;
        float y = -PI / 2;
        float z = -x + atan2(-R[1][0], -R[2][0]);
        return float3 { x, y, z };
    } else { //two solutions exist
        float x1 = -asin(R[0][2]);
        float x2 = PI - x1;

        float y1 = atan2(R[1][2] / cos(x1), R[2][2] / cos(x1));
        float y2 = atan2(R[1][2] / cos(x2), R[2][2] / cos(x2));

        float z1 = atan2(R[0][1] / cos(x1), R[0][0] / cos(x1));
        float z2 = atan2(R[0][1] / cos(x2), R[0][0] / cos(x2));

        //choose one solution to return
        //for example the "shortest" rotation
        if ((std::abs(x1) + std::abs(y1) + std::abs(z1)) <= (std::abs(x2) + std::abs(y2) + std::abs(z2))) {
            return float3 { x1, y1, z1 };
        } else {
            return float3 { x2, y2, z2 };
        }
    }
}

void print_array(float3 myarray){
	for (int i = 3 - 1; i >= 0; i--)
	    std::cout << myarray[i];
}

/* ========================================== *\
 * 		ITERATIVELY CLOSEST POINT
\* ========================================== */

void iterative_closest_point(std::string path_cloud1, std::string path_cloud2)
{

	// declare two point clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);

    // specify file path
	std::string path = ros::package::getPath("dyn_3d_mod");

	// read clouds from ply files
	pcl::io::loadPLYFile(path+"/recordings/monkey.ply", *cloud1);
	pcl::io::loadPLYFile(path+"/recordings/monkey_rotated.ply", *cloud2);

	// ====================== ICP

	// use iterative closet point
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	// final result
	pcl::PointCloud<pcl::PointXYZ> Final;

	// do alignment
	icp.setMaximumIterations (100);
	//icp.setMaxCorrespondenceDistance (0.1);
	icp.setInputSource(cloud1);
	icp.setInputTarget(cloud2);
	icp.align(Final);


	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;

	// get transformation
	Eigen::Matrix4f eigenm = icp.getFinalTransformation ();

	std::cout << eigenm << std::endl;


	// apply found transformation
	//pcl::transformPointCloud (*input_cloud, *output_cloud, icp.getFinalTransformation ());

	// save result
	//pcl::io::savePCDFileASCII ("output.pcd", Final);
	//pcl::io::savePCDFileBinary ("output.pcd", Final);

	// ====================== VISUALIZATION

	pcl::visualization::PCLVisualizer viewer_ ("ICP");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud1, 0, 0, 255);
	viewer_.addPointCloud (cloud1, source_cloud_color_handler, "original_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (cloud2, 230, 20, 20);
	viewer_.addPointCloud (cloud2, transformed_cloud_color_handler, "transformed_cloud");

	while (!viewer_.wasStopped ()) { // Display the visualiser until 'q' key is pressed
		viewer_.spinOnce ();
	}

}


/* ========================================== *\
 * 		MAIN FUNCTION
\* ========================================== */


int main(int argc, char** argv)
{

	if(argc < 3){
		std::cout << "Please select two point clouds." << std::endl;
		return -1;
	}

	iterative_closest_point(argv[1], argv[2]);

	return 0;

}

