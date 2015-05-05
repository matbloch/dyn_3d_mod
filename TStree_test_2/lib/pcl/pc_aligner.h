#include <iostream>

// PCL specific includes
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
#include <pcl/filters/passthrough.h>

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

    // type definitions
	typedef pcl::PointCloud<pcl::PointXYZ> PCXYZ;
	typedef pcl::PointCloud<pcl::Normal> NORMAL;
	typedef pcl::PointCloud<pcl::SHOT352> SHOTFEATURE;

	PCAligner();
	~PCAligner();

    void setInputClouds (PCXYZ::Ptr c1, PCXYZ::Ptr c2);

    /* parameters */
    void setCutoffDistance(float dist);
    void setLeafSize(float size);
    void setNormalEstSearchRadius(float radius);
    void setRANSACMaxCorrespondenceDist(float dist);
    void setSHOTSearchRadius(float radius);
    void setICPMaximumIterations(int nr_iter);
    void setICPMaximumCorrelationDist(float dist);

    /* data extraction */
    Eigen::Matrix4f getFinalTransformation();


    void startAlignment ()
    {

    	// validate parameters
    	if(!validateParameters()){
    		return;
    	}

    	// preprocessing
		filterClouds();

		// prealignment
		computeNormals();
		detectSHOTFeatures();
		matchSHOTDescriptors();

		if(SampleConsensus() != 0){
			return;
		}

		// refinement
		ICPRefinement();
    }

  protected:

    /* validation */
    bool validateParameters();

    /* processing functions */
    void filterClouds ();
    void computeNormals ();
    void detectSHOTFeatures ();
    void matchSHOTDescriptors();
    int SampleConsensus();
    int ICPRefinement();

  private:

	// input clouds
	PCXYZ::Ptr cloud1_;
	PCXYZ::Ptr cloud2_;

	// downsampled clouds for RANSAC approximation
	PCXYZ::Ptr filtered_estimation_cloud1_;
	PCXYZ::Ptr filtered_estimation_cloud2_;


	// normals of the filtered clouds
	NORMAL::Ptr normals1_;
	NORMAL::Ptr normals2_;

	// SHOT features of the filtered point clouds
	SHOTFEATURE::Ptr shot_descriptors1_;
	SHOTFEATURE::Ptr shot_descriptors2_;

	// aligned clouds (starting from cloud1)
	PCXYZ::Ptr ransac_aligned_cloud1_;
	PCXYZ::Ptr icp_aligned_cloud1_;

	// calculated transformations (RANSAC->ICP)
	Eigen::Matrix4f ransac_transformation_;
	Eigen::Matrix4f icp_transformation_;

    // Parameters
	float cut_off_distance_;
	float leaf_size_;	// downsampling size
	float normal_est_search_radius_;
	float shot_search_radius_;
	float ransac_max_corr_dist_;
	int icp_max_iter_;
	float icp_max_corr_dist_;

};

/* ========================================== *\
 * 		FORWARD DECLARATIONS
\* ========================================== */


PCAligner::PCAligner () :
	  cut_off_distance_ (100),
      leaf_size_ (0.2),
      normal_est_search_radius_ (1),
      shot_search_radius_ (0.8),
      ransac_max_corr_dist_ (0.2),
      icp_max_iter_(100),
      icp_max_corr_dist_(0.2)

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

}

void PCAligner::setCutoffDistance(float dist){
	cut_off_distance_ = dist;
}
void PCAligner::setLeafSize(float size){
	leaf_size_ = size;
}
void PCAligner::setNormalEstSearchRadius(float radius){
	normal_est_search_radius_ = radius;
}
void PCAligner::setRANSACMaxCorrespondenceDist(float dist){
	ransac_max_corr_dist_ = dist;
}
void PCAligner::setSHOTSearchRadius(float radius){
	shot_search_radius_ = radius;
}
void PCAligner::setICPMaximumIterations(int nr_iter){
	icp_max_iter_ = nr_iter;
}
void PCAligner::setICPMaximumCorrelationDist(float dist){
	icp_max_corr_dist_ = dist;
}

bool PCAligner::validateParameters(){

	// check if downsampling leafsize for approximation is feasible
	if(leaf_size_*1.1 >= normal_est_search_radius_){
		std::cout << "Normal search radius is too small: The normal search radius needs to be at least the size of the downsampling leaf size" << std::endl;
		return false;
	}

	return true;

}


/* ========================================== *\
 * 		1. DOWNSAMPLING
\* ========================================== */

void PCAligner::filterClouds ()
{

	filtered_estimation_cloud1_ = PCXYZ::Ptr (new PCXYZ);
	filtered_estimation_cloud2_ = PCXYZ::Ptr (new PCXYZ);

	// 1: CUTOFF FILTERING - removing measurements with big variance
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0, cut_off_distance_);

	pass.setInputCloud (cloud1_);
	pass.filter (*filtered_estimation_cloud1_);

	pass.setInputCloud (cloud2_);
	pass.filter (*filtered_estimation_cloud2_);


	// 2: DOWNSAMPLING
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	filter.setLeafSize(leaf_size_,leaf_size_,leaf_size_);


	// filter first cloud
	filter.setInputCloud(cloud1_);
	filter.filter(*filtered_estimation_cloud1_);

	// filter second cloud
	filter.setInputCloud(cloud2_);
	filter.filter(*filtered_estimation_cloud2_);

	std::cout << "--- downsampling complete" << std::endl;

}

/* ========================================== *\
 * 		2. NORMAL CALCULATION
\* ========================================== */

void PCAligner::computeNormals(){

	normals1_ = NORMAL::Ptr (new NORMAL);
	normals2_ = NORMAL::Ptr (new NORMAL);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	norm_est.setSearchMethod(kdtree);
	norm_est.setRadiusSearch (normal_est_search_radius_);

	// cloud 1
	norm_est.setInputCloud (filtered_estimation_cloud1_);
	norm_est.compute (*normals1_);

	// cloud 2
	norm_est.setInputCloud (filtered_estimation_cloud2_);
	norm_est.compute (*normals2_);

	// Visualize them.
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	viewer->addPointCloud<pcl::PointXYZ>(filtered_estimation_cloud1_, "cloud");
	// Display one normal out of 20, as a line of length 3cm.
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(filtered_estimation_cloud1_, normals1_, 4, 0.2, "normals");
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
	shot_est.setInputCloud(filtered_estimation_cloud1_);
	shot_est.setInputNormals(normals1_);
	shot_est.compute(*shot_descriptors1_);

	// cloud 2
	shot_est.setInputCloud(filtered_estimation_cloud2_);
	shot_est.setInputNormals(normals2_);
	shot_est.compute(*shot_descriptors2_);

	std::cout << "--- SHOT feature search complete" << std::endl;

	// apply transformation on original cloud
	ransac_aligned_cloud1_ = PCXYZ::Ptr (new PCXYZ);
	pcl::transformPointCloud (*cloud1_, *ransac_aligned_cloud1_, ransac_transformation_);


	// visualize sub result
	pcl::visualization::PCLVisualizer viewer_ ("Features");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud1_, 0, 0, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (cloud2_, 230, 20, 20);

	viewer_.addPointCloud (filtered_estimation_cloud1_, source_cloud_color_handler, "original cloud");
	viewer_.addPointCloud (filtered_estimation_cloud2_, transformed_cloud_color_handler, "RANSAC aligned cloud");

	// add features
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp_shot (new pcl::PointCloud<pcl::PointXYZ>);
	copyPointCloud(*shot_descriptors2_, *cloud_temp_shot);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> shot_keypoints_color_handler (cloud_temp_shot, 0, 255, 0);
	viewer_.addPointCloud (cloud_temp_shot, shot_keypoints_color_handler, "shot features");
	viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "shot features");


	while (!viewer_.wasStopped ()) { // Display the visualiser until 'q' key is pressed
		viewer_.spinOnce ();
	}

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

	ransac.setInputSource(filtered_estimation_cloud1_);	// calculate transformation from alignment of filtered cloud 1 to filtered cloud 2
	ransac.setSourceFeatures(shot_descriptors1_);
	ransac.setInputTarget(filtered_estimation_cloud2_);
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

	// specify parameters
	icp.setMaximumIterations (icp_max_iter_);
	icp.setMaxCorrespondenceDistance (icp_max_corr_dist_);
	icp.setRANSACOutlierRejectionThreshold (0.6);

	icp.setInputSource(ransac_aligned_cloud1_);
	icp.setInputTarget(cloud2_);

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

/* ========================================== *\
 * 		DATA EXTRACTION
\* ========================================== */

Eigen::Matrix4f PCAligner::getFinalTransformation(){

	return icp_transformation_*ransac_transformation_;

}
