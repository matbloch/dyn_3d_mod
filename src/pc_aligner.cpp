#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>

// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>

// features
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/shot.h>

// filtering/downsampling
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// correspondences
#include <pcl/correspondence.h>
#include <pcl/recognition/cg/geometric_consistency.h>

// RANSAC
#include <pcl/registration/sample_consensus_prerejective.h>

// ICP
#include <pcl/registration/icp.h>

int main(int argc, char** argv)
{

	/*
	 * WORKPACKAGES
	 *
	 * 1. Downsample point clouds
	 * 2. Calculate normals
	 * 3. Detect SHOT features
	 * 4. Match descriptors
	 * 5. Estimate transformation using RANSAC
	 * 6. Apply approximated transformation to original cloud
	 * 7. Refine alignment using ICP
	 * 8. Combine transformations
	 */

	/* ========================================== *\
	 * 		HELP
	\* ========================================== */

	if(argc<4 || argv[1] == std::string("help")){
		std::cout<<"\n";
		std::cout << "=================================" << std::endl;
		std::cout << " USAGE:" << std::endl;
		std::cout << "---------------------------------" << std::endl;
		std::cout << " --file1 (relative to /recordings)" << std::endl;
		std::cout << " --file2 (relative to /recordings)" << std::endl;
		std::cout << " --leaf_size (in meters)" << std::endl;
		std::cout << "=================================" << std::endl;
		std::cout<<"\n";
	}

	/* ========================================== *\
	 * 		LOAD CLOUDS
	\* ========================================== */

	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(ros::package::getPath("dyn_3d_mod") + "/recordings/" + argv[1], *cloud1) != 0)
	{
		return -1;
	}
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(ros::package::getPath("dyn_3d_mod") + "/recordings/" + argv[2], *cloud2) != 0)
	{
		return -1;
	}

	/* ========================================== *\
	 * 		1. DOWNSAMPLING
	\* ========================================== */

	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud2(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::VoxelGrid<pcl::PointXYZ> filter;


	float leaf_size;	// leaf size in m (only one point per {leaf_size} every cubic m will survive).
	leaf_size = std::atof(argv[3]);
	std::cout << "--- Setting leaf size to " << argv[3] << " meters" << std::endl;

	filter.setLeafSize(leaf_size,leaf_size,leaf_size);

	// filter first cloud
	filter.setInputCloud(cloud1);
	filter.filter(*filteredCloud1);

	// filter second cloud
	filter.setInputCloud(cloud2);
	filter.filter(*filteredCloud2);

	std::cout << "--- downsampling complete" << std::endl;

	/* ========================================== *\
	 * 		2. NORMAL CALUCLATION
	\* ========================================== */

	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals1(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>);

	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setRadiusSearch(1);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);

	// cloud 1
	normalEstimation.setInputCloud(filteredCloud1);
	normalEstimation.compute(*normals1);

	// cloud 2
	normalEstimation.setInputCloud(filteredCloud2);
	normalEstimation.compute(*normals2);
	std::cout << "--- normal calculation complete" << std::endl;

	/* ========================================== *\
	 * 		3. SHOT FEATURE DETECTION
	\* ========================================== */

	pcl::PointCloud<pcl::SHOT352>::Ptr descriptors1(new pcl::PointCloud<pcl::SHOT352>());
	pcl::PointCloud<pcl::SHOT352>::Ptr descriptors2(new pcl::PointCloud<pcl::SHOT352>());

	pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
	shot.setRadiusSearch(0.8); // The radius that defines which of the keypoint's neighbors are described. If too large, there may be clutter, and if too small, not enough points may be found.

	// cloud 1
	shot.setInputCloud(filteredCloud1);
	shot.setInputNormals(normals1);
	shot.compute(*descriptors1);

	// cloud 2
	shot.setInputCloud(filteredCloud2);
	shot.setInputNormals(normals2);
	shot.compute(*descriptors2);

	std::cout << "--- feature search complete" << std::endl;

	/* ========================================== *\
	 * 		4. DESCRIPTOR MATCHING
	\* ========================================== */

	// A kd-tree object that uses the FLANN library for fast search of nearest neighbors.
	pcl::KdTreeFLANN<pcl::SHOT352> matching;
	matching.setInputCloud(descriptors1);

	// A Correspondence object stores the indices of the query and the match,
	// and the distance/weight.
	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());

	// Check every descriptor computed for the filteredCloud2.
	for (size_t i = 0; i < descriptors2->size(); ++i)
	{
		std::vector<int> neighbors(1);
		std::vector<float> squaredDistances(1);
		// Ignore NaNs.
		if (pcl_isfinite(descriptors2->at(i).descriptor[0]))
		{
			// Find the nearest neighbor (in descriptor space)...
			int neighborCount = matching.nearestKSearch(descriptors2->at(i), 1, neighbors, squaredDistances);
			// ...and add a new correspondence if the distance is less than a threshold
			// (SHOT distances are between 0 and 1, other descriptors use different metrics).
			if (neighborCount == 1 && squaredDistances[0] < 0.25f)
			{
				pcl::Correspondence correspondence(neighbors[0], static_cast<int>(i), squaredDistances[0]);
				correspondences->push_back(correspondence);
			}
		}
	}

	std::cout << "--- descriptor matching complete. found " << correspondences->size() << " correspondences." << std::endl;

	/* ========================================== *\
	 * 		5. CORRESPONDENCE REJECTOR SAMPLE CONSENSUS
	\* ========================================== */

	pcl::PointCloud<pcl::PointXYZ>::Ptr alignedfilteredCloud1(new pcl::PointCloud<pcl::PointXYZ>);


	pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::SHOT352> pose;
	pose.setInputSource(filteredCloud1);
	pose.setInputTarget(filteredCloud2);
	pose.setSourceFeatures(descriptors1);
	pose.setTargetFeatures(descriptors2);
	// Instead of matching a descriptor with its nearest neighbor, choose randomly between
	// the N closest ones, making it more robust to outliers, but increasing time.
	pose.setCorrespondenceRandomness(2);
	// Set the fraction (0-1) of inlier points required for accepting a transformation.
	// At least this number of points will need to be aligned to accept a pose.
	pose.setInlierFraction(0.25f);
	// Set the number of samples to use during each iteration (minimum for 6 DoF is 3).
	pose.setNumberOfSamples(3);
	// Set the similarity threshold (0-1) between edge lengths of the polygons. The
	// closer to 1, the more strict the rejector will be, probably discarding acceptable poses.
	pose.setSimilarityThreshold(0.6f);
	// Set the maximum distance threshold between two correspondent points in source and target.
	// If the distance is larger, the points will be ignored in the alignment process.
	pose.setMaxCorrespondenceDistance(0.2f);

	pose.align(*alignedfilteredCloud1);


	Eigen::Matrix4f transformation;

	if (pose.hasConverged())
	{
		transformation = pose.getFinalTransformation();
		Eigen::Matrix3f rotation = transformation.block<3, 3>(0, 0);
		Eigen::Vector3f translation = transformation.block<3, 1>(0, 3);

		std::cout << "Transformation matrix:" << std::endl << std::endl;
		printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
		printf("\t\tR = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
		printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
		std::cout << std::endl;
		printf("\t\tt = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
	}
	else{
		std::cout << "--- RANSAC did not converge." << std::endl;
		return -1;
	}

	/* ========================================== *\
	 * 		6. APPLY TRANSFORMATION ON ORIGINAL CLOUD
	\* ========================================== */

	  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	  pcl::transformPointCloud (*filteredCloud1, *transformed_cloud, transformation);

	/* ========================================== *\
	 * 		VISUALIZE SUBRESULT
	\* ========================================== */

	pcl::visualization::PCLVisualizer viewer_ ("Prealignement using SHOT feature matching & RANSAC");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud1, 0, 0, 255);
	viewer_.addPointCloud (filteredCloud2, source_cloud_color_handler, "original_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (cloud2, 230, 20, 20);
	viewer_.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

	while (!viewer_.wasStopped ()) { // Display the visualiser until 'q' key is pressed
		viewer_.spinOnce ();
	}

	/* ========================================== *\
	 * 		7. ICP REFINEMENT
	\* ========================================== */

	// use iterative closet point
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	// final result
	pcl::PointCloud<pcl::PointXYZ> Final;

	// do alignment
	icp.setMaximumIterations (100);
	//icp.setMaxCorrespondenceDistance (0.1);
	icp.setInputSource(transformed_cloud);
	icp.setInputTarget(filteredCloud2);
	icp.align(Final);


	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;


	Eigen::Matrix4f transformation2;

	if (icp.hasConverged())
	{
		transformation2 = icp.getFinalTransformation ();
		Eigen::Matrix3f rotation2 = transformation2.block<3, 3>(0, 0);
		Eigen::Vector3f translation2 = transformation2.block<3, 1>(0, 3);

		std::cout << "Transformation matrix:" << std::endl << std::endl;
		printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation2(0, 0), rotation2(0, 1), rotation2(0, 2));
		printf("\t\tR = | %6.3f %6.3f %6.3f | \n", rotation2(1, 0), rotation2(1, 1), rotation2(1, 2));
		printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation2(2, 0), rotation2(2, 1), rotation2(2, 2));
		std::cout << std::endl;
		printf("\t\tt = < %0.3f, %0.3f, %0.3f >\n", translation2(0), translation2(1), translation2(2));
	}
	else{
		std::cout << "ICP Did not converge." << std::endl;
		return -1;
	}

	/* ========================================== *\
	 * 		APPLY 2nd TRANSFORMATION
	\* ========================================== */

	  // Executing the transformation
	  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud2 (new pcl::PointCloud<pcl::PointXYZ> ());

	  // apply both transformation to the original cloud
	  pcl::transformPointCloud (*transformed_cloud, *transformed_cloud2, transformation2);


	/* ========================================== *\
	 * 		VISUALIZE END RESULT
	\* ========================================== */

	pcl::visualization::PCLVisualizer viewer2_ ("END RESULT");

	viewer2_.addPointCloud (filteredCloud2, source_cloud_color_handler, "original_cloud");
	viewer2_.addPointCloud (transformed_cloud2, transformed_cloud_color_handler, "transformed_cloud");

	while (!viewer2_.wasStopped ()) {
		viewer2_.spinOnce ();
	}

	/* ========================================== *\
	 * 		8. COMBINE TRANSFORMATIONS
	\* ========================================== */



	return 0;

}
