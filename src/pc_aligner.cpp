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

class PCAligner
{
  public:
    // A bit of shorthand

    // type definitions
	typedef pcl::PointCloud<pcl::PointXYZ> PCXYZ;
	typedef pcl::PointCloud<pcl::Normal> normal;
	typedef pcl::PointCloud<pcl::SHOT352> SHOTFEATURE;


	PCAligner();
	~PCAligner();


    void setInputClouds (PCXYZ::Ptr c1, PCXYZ::Ptr c2);

  protected:

    /* processing functions */
    void startAlignment ()
    {

		filterClouds();
		computeNormals ();
		detectSHOTFeatures();
		matchSHOTDescriptors();

		if(SampleConsensus() != 0){
			return;
		}

		ICPRefinement();
    }

    void filterClouds ();
    void computeNormals ();
    void detectSHOTFeatures ();
    void matchSHOTDescriptors();
    int SampleConsensus();
    int ICPRefinement();

    /* data extraction */
    Eigen::Matrix4f getFinalTransformation();


  private:

	// input clouds
	PCXYZ::Ptr cloud1_;
	PCXYZ::Ptr cloud2_;

	// downsampled clouds
	PCXYZ::Ptr filtered_cloud1_;
	PCXYZ::Ptr filtered_cloud2_;

	// normals of the filtered clouds
	normal::Ptr normals1_;
	normal::Ptr normals2_;

	// SHOT features of the filtered point clouds
	SHOTFEATURE::Ptr shot_descriptors1_;
	SHOTFEATURE::Ptr shot_descriptors2_;

	// aligned clouds
	PCXYZ::Ptr ransac_aligned_cloud1_;
	PCXYZ::Ptr icp_aligned_cloud1_;

	// calculated transformations
	Eigen::Matrix4f ransac_transformation_;
	Eigen::Matrix4f icp_transformation_;

    // Parameters
	float leaf_size_;	// downsampling size
	float normal_est_search_radius_;
	float shot_search_radius_;

};

/* ========================================== *\
 * 		FORWARD DECLARATIONS
\* ========================================== */


PCAligner::PCAligner () :
      leaf_size_ (0.2),
      normal_est_search_radius_ (1),
      shot_search_radius_ (0.8)
    {

}
PCAligner::~PCAligner (){

}

void PCAligner::setInputClouds (PCXYZ::Ptr c1, PCXYZ::Ptr c2)
{
	cloud1_ = c1;
	cloud2_ = c2;

	//remove NAN points from the cloud
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud1_,*cloud1_, indices);
	pcl::removeNaNFromPointCloud(*cloud2_,*cloud2_, indices);

	startAlignment ();
}

/* ========================================== *\
 * 		1. DOWNSAMPLING
\* ========================================== */

void PCAligner::filterClouds ()
{


	filtered_cloud1_ = PCXYZ::Ptr (new PCXYZ);
	filtered_cloud2_ = PCXYZ::Ptr (new PCXYZ);

	pcl::VoxelGrid<pcl::PointXYZ> filter;
	filter.setLeafSize(leaf_size_,leaf_size_,leaf_size_);


	// filter first cloud
	filter.setInputCloud(cloud1_);
	filter.filter(*filtered_cloud1_);

	// filter second cloud
	filter.setInputCloud(cloud2_);
	filter.filter(*filtered_cloud2_);

	std::cout << "--- downsampling complete" << std::endl;

}

/* ========================================== *\
 * 		2. NORMAL CALCULATION
\* ========================================== */

void PCAligner::computeNormals(){

	normals1_ = normal::Ptr (new normal);
	normals2_ = normal::Ptr (new normal);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	norm_est.setSearchMethod(kdtree);
	norm_est.setRadiusSearch (normal_est_search_radius_);

	// cloud 1
	norm_est.setInputCloud (filtered_cloud1_);
	norm_est.compute (*normals1_);

	// cloud 2
	norm_est.setInputCloud (filtered_cloud2_);
	norm_est.compute (*normals2_);

	// Visualize them.
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	viewer->addPointCloud<pcl::PointXYZ>(filtered_cloud1_, "cloud");
	// Display one normal out of 20, as a line of length 3cm.
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(filtered_cloud1_, normals1_, 4, 0.2, "normals");
	while (!viewer->wasStopped())
	{
	viewer->spinOnce(100);
	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

}

/* ========================================== *\
 * 		3. SHOT FEATURE DETECTION
\* ========================================== */

void PCAligner::detectSHOTFeatures ()
{

	pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot_est;
	shot_est.setRadiusSearch(shot_search_radius_);

	shot_descriptors1_ = SHOTFEATURE::Ptr (new SHOTFEATURE);
	shot_descriptors2_ = SHOTFEATURE::Ptr (new SHOTFEATURE);

	// cloud 1
	shot_est.setInputCloud(filtered_cloud1_);
	shot_est.setInputNormals(normals1_);
	shot_est.compute(*shot_descriptors1_);

	// cloud 2
	shot_est.setInputCloud(filtered_cloud2_);
	shot_est.setInputNormals(normals2_);
	shot_est.compute(*shot_descriptors2_);

	std::cout << "--- feature search complete" << std::endl;

}


/* ========================================== *\
 * 		4. DESCRIPTOR MATCHING
\* ========================================== */

void PCAligner::matchSHOTDescriptors (){

	// A kd-tree object that uses the FLANN library for fast search of nearest neighbors.
	pcl::KdTreeFLANN<pcl::SHOT352> matching;
	matching.setInputCloud(shot_descriptors1_);

	// A Correspondence object stores the indices of the query and the match,
	// and the distance/weight.
	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());

	// Check every descriptor computed for the filteredCloud2.
	for (size_t i = 0; i < shot_descriptors2_->size(); ++i)
	{
		std::vector<int> neighbors(1);
		std::vector<float> squaredDistances(1);
		// Ignore NaNs.
		if (pcl_isfinite(shot_descriptors2_->at(i).descriptor[0]))
		{
			// Find the nearest neighbor (in descriptor space)...
			int neighborCount = matching.nearestKSearch(shot_descriptors2_->at(i), 1, neighbors, squaredDistances);
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
}


/* ========================================== *\
 * 		5. SAMPLE CONSENSUS
\* ========================================== */

int PCAligner::SampleConsensus(){

	PCXYZ::Ptr aligned_filtered_cloud1 = PCXYZ::Ptr (new PCXYZ);

	pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::SHOT352> ransac;

	ransac.setInputSource(filtered_cloud1_);	// calculate transformation from alignment of filtered cloud 1 to filtered cloud 2
	ransac.setSourceFeatures(shot_descriptors1_);
	ransac.setInputTarget(filtered_cloud2_);
	ransac.setTargetFeatures(shot_descriptors2_);

	// Instead of matching a descriptor with its nearest neighbor, choose randomly between
	// the N closest ones, making it more robust to outliers, but increasing time.
	ransac.setCorrespondenceRandomness(2);
	// Set the fraction (0-1) of inlier points required for accepting a transformation.
	// At least this number of points will need to be aligned to accept a pose.
	ransac.setInlierFraction(0.25f);
	// Set the number of samples to use during each iteration (minimum for 6 DoF is 3).
	ransac.setNumberOfSamples(3);
	// Set the similarity threshold (0-1) between edge lengths of the polygons. The
	// closer to 1, the more strict the rejector will be, probably discarding acceptable poses.
	ransac.setSimilarityThreshold(0.6f);
	// Set the maximum distance threshold between two correspondent points in source and target.
	// If the distance is larger, the points will be ignored in the alignment process.
	ransac.setMaxCorrespondenceDistance(0.2f);

	ransac.align(*aligned_filtered_cloud1); // BEFORE ransac.hasConverged() !

	if (ransac.hasConverged())
	{
		ransac_transformation_ = ransac.getFinalTransformation();
		Eigen::Matrix3f rotation = ransac_transformation_.block<3, 3>(0, 0);
		Eigen::Vector3f translation = ransac_transformation_.block<3, 1>(0, 3);

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


	// apply transformation on original cloud
	ransac_aligned_cloud1_ = PCXYZ::Ptr (new PCXYZ);
	pcl::transformPointCloud (*cloud1_, *ransac_aligned_cloud1_, ransac_transformation_);


	// visualize sub result
	pcl::visualization::PCLVisualizer viewer_ ("Prealignement using SHOT feature matching & RANSAC");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud1_, 0, 0, 255);
	viewer_.addPointCloud (cloud2_, source_cloud_color_handler, "original cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (cloud2_, 230, 20, 20);
	viewer_.addPointCloud (ransac_aligned_cloud1_, transformed_cloud_color_handler, "RANSAC aligned cloud");

	while (!viewer_.wasStopped ()) { // Display the visualiser until 'q' key is pressed
		viewer_.spinOnce ();
	}

	return 0;

}


/* ========================================== *\
 * 		7. ICP REFINEMENT
\* ========================================== */

int PCAligner::ICPRefinement(){


	// use iterative closet point
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	// do alignment
	icp.setMaximumIterations (100);
	//icp.setMaxCorrespondenceDistance (0.1);
	icp.setInputSource(ransac_aligned_cloud1_);
	icp.setInputTarget(cloud2_);


	/* only consider points up to 3.5m */
	/*
	icp.setFilterFieldName("z");
	icp.setFilterLimits(3.5);
	*/

	// align clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud2 (new pcl::PointCloud<pcl::PointXYZ> ());
	icp.align(*transformed_cloud2); // BEFORE pose.hasConverged() !


	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;

	if (icp.hasConverged())
	{
		icp_transformation_ = icp.getFinalTransformation ();
		Eigen::Matrix3f rotation2 = icp_transformation_.block<3, 3>(0, 0);
		Eigen::Vector3f translation2 = icp_transformation_.block<3, 1>(0, 3);

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


	// apply both transformation to the original cloud
	icp_aligned_cloud1_ = PCXYZ::Ptr (new PCXYZ);
	pcl::transformPointCloud (*ransac_aligned_cloud1_, *icp_aligned_cloud1_, icp_transformation_);


	// visualize
	pcl::visualization::PCLVisualizer viewer2_ ("END RESULT");


	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud1_, 0, 0, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (cloud2_, 230, 20, 20);


	viewer2_.addPointCloud (cloud2_, source_cloud_color_handler, "original_cloud");
	viewer2_.addPointCloud (icp_aligned_cloud1_, transformed_cloud_color_handler, "transformed_cloud");

	while (!viewer2_.wasStopped ()) {
		viewer2_.spinOnce ();
	}

	return 0;

}


Eigen::Matrix4f PCAligner::getFinalTransformation(){

	return icp_transformation_*ransac_transformation_;

}


int main (int argc, char **argv)
{


	  // Load the target cloud PCD file
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::io::loadPCDFile (ros::package::getPath("dyn_3d_mod") + "/recordings/cloud1.pcd", *cloud1);
	  pcl::io::loadPCDFile (ros::package::getPath("dyn_3d_mod") + "/recordings/cloud2.pcd", *cloud2);

	  PCAligner target_cloud;
	  //target_cloud.setInputCloud (cloud1);
	  target_cloud.setInputClouds (cloud1, cloud2);


  return (0);
}
