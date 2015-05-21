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

// outlier removal
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/median_filter.h>

class PCAligner
{
	/*
	 * DESCRIPTION:
	 * ===================================================================
	 * Calculates the transformation to align cloud1_ with cloud2_.
	 * The transformation maps cloud1_ to the coordinate system of cloud2_
	 */

  public:

    // type definitions
	typedef pcl::PointCloud<pcl::PointXYZ> PCXYZ;
	typedef pcl::PointCloud<pcl::Normal> NORMAL;
	typedef pcl::PointCloud<pcl::SHOT352> SHOTFEATURE;

	PCAligner();
	~PCAligner();

    void setInputClouds (PCXYZ::Ptr c1, PCXYZ::Ptr c2);

    /* parameters */
    void setOutlierRemovalNeighbourhood(int);
    void setOutlierRemovalStddev(float);
    void setCutoffDistance(std::vector<float> thresholds);
    void setLeafSize(float size);
    void setNormalEstSearchRadius(float radius);
    void setRANSACMaxCorrespondenceDist(float dist);
    void setSHOTSearchRadius(float radius);
    void setICPMaximumIterations(int nr_iter);
    void setICPMaximumCorrelationDist(float dist);

	void displaySubResults(bool);
	void displayEndResult(bool);

    /* data extraction */
    Eigen::Matrix4f getFinalTransformation();
    void printRotTrans(Eigen::Matrix4f);

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
	int outlier_removal_neighbourhood_;
	float outlier_removal_stddev_;
	float cut_off_distance_x_from_;
	float cut_off_distance_x_to_;
	float cut_off_distance_y_from_;
	float cut_off_distance_y_to_;
	float cut_off_distance_z_;
	float leaf_size_;	// downsampling size
	float normal_est_search_radius_;
	float shot_search_radius_;
	float ransac_max_corr_dist_;
	int icp_max_iter_;
	float icp_max_corr_dist_;

	// Options
	bool display_sub_results_;
	bool display_end_result_;


};

/* ========================================== *\
 * 		FORWARD DECLARATIONS
\* ========================================== */


PCAligner::PCAligner () :
	  outlier_removal_neighbourhood_(50),
	  outlier_removal_stddev_ (1.0),
	  cut_off_distance_x_from_ (0),
	  cut_off_distance_x_to_ (0),
	  cut_off_distance_y_from_ (0),
	  cut_off_distance_y_to_ (0),
	  cut_off_distance_z_ (15),
      leaf_size_ (0.2),
      normal_est_search_radius_ (1),
      shot_search_radius_ (0.8),
      ransac_max_corr_dist_ (0.2),
      icp_max_iter_(100),
      icp_max_corr_dist_(0.2),
      display_sub_results_(false),
      display_end_result_(true)
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

void PCAligner::setOutlierRemovalNeighbourhood(int size){
	outlier_removal_neighbourhood_ = size;
}
void PCAligner::setOutlierRemovalStddev(float stddev){
	outlier_removal_stddev_ = stddev;
}

void PCAligner::setCutoffDistance(std::vector<float> thresholds){


	if(thresholds.size() != 5){
		std::cout << "The thresholds must have the following format: {x_from, x_to, y_from, y_to, z}. A threshold equal to zero means no filtering.";
		return;
	}


	cut_off_distance_x_from_ = thresholds[0];
	cut_off_distance_x_to_ = thresholds[1];
	cut_off_distance_y_from_ = thresholds[2];
	cut_off_distance_y_to_ = thresholds[3];
	cut_off_distance_z_ = thresholds[4];
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

void PCAligner::displaySubResults(bool disp){
	display_sub_results_ = disp;
}
void PCAligner::displayEndResult(bool disp){
	display_end_result_ = disp;
}


/* ========================================== *\
 * 		1. DOWNSAMPLING
\* ========================================== */

void PCAligner::filterClouds ()
{

	filtered_estimation_cloud1_ = PCXYZ::Ptr (new PCXYZ);
	filtered_estimation_cloud2_ = PCXYZ::Ptr (new PCXYZ);
	PCXYZ::Ptr tmp1 = PCXYZ::Ptr (new PCXYZ);
	PCXYZ::Ptr tmp2 = PCXYZ::Ptr (new PCXYZ);


	// 0: outlier removal
	if(outlier_removal_neighbourhood_ > 0){
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setMeanK (50);	// size of the neighbourhood pointset
		sor.setStddevMulThresh (1.0);	// points with distance larger than std dev of mean distance will be removed

		std::cout << "Nr. points before outlier removal: cloud1: " << cloud1_->points.size () << ", cloud2: " << cloud2_->points.size () << std::endl;

		sor.setInputCloud (cloud1_);
		sor.filter (*cloud1_);

		sor.setInputCloud (cloud2_);
		sor.filter (*cloud2_);
		std::cout << "Nr. points after outlier removal: cloud1: " << cloud1_->points.size () << ", cloud2: " << cloud2_->points.size () << std::endl;
	}

	// 1: CUTOFF FILTERING - removing measurements with big variance
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0, cut_off_distance_z_);

	pass.setInputCloud (cloud1_);
	pass.filter (*filtered_estimation_cloud1_);

	pass.setInputCloud (cloud2_);
	pass.filter (*filtered_estimation_cloud2_);

	if(cut_off_distance_x_from_!= cut_off_distance_x_to_){
		pass.setFilterFieldName ("x");
		pass.setFilterLimits (cut_off_distance_x_from_, cut_off_distance_x_to_);
		pass.setInputCloud (filtered_estimation_cloud1_);
		pass.filter (*filtered_estimation_cloud1_);
		pass.setInputCloud (filtered_estimation_cloud2_);
		pass.filter (*filtered_estimation_cloud2_);
	}
	if(cut_off_distance_y_from_!=cut_off_distance_y_to_){
		pass.setFilterFieldName ("y");
		pass.setFilterLimits (cut_off_distance_y_from_, cut_off_distance_y_to_);
		pass.setInputCloud (filtered_estimation_cloud1_);
		pass.filter (*filtered_estimation_cloud1_);
		pass.setInputCloud (filtered_estimation_cloud2_);
		pass.filter (*filtered_estimation_cloud2_);
	}

	// 2: DOWNSAMPLING
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	filter.setLeafSize(leaf_size_,leaf_size_,leaf_size_);

	// filter first cloud
	filter.setInputCloud(filtered_estimation_cloud1_);
	filter.filter(*tmp1);

	filtered_estimation_cloud1_ = tmp1;

	// filter second cloud
	filter.setInputCloud(filtered_estimation_cloud2_);
	filter.filter(*tmp2);

	filtered_estimation_cloud2_ = tmp2;

	std::cout << "--- downsampling and filtering complete." << std::endl;


	// visualize sub result
	if(display_sub_results_)
	{
		pcl::visualization::PCLVisualizer viewer_ ("Filtered clouds");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud1_, 0, 0, 255);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (cloud2_, 230, 20, 20);
		viewer_.addPointCloud (filtered_estimation_cloud1_, source_cloud_color_handler, "Cloud1");
		viewer_.addPointCloud (filtered_estimation_cloud2_, transformed_cloud_color_handler, "Cloud2");
		viewer_.addCoordinateSystem (1.0);

		while (!viewer_.wasStopped ()) { // Display the visualiser until 'q' key is pressed
			viewer_.spinOnce ();
		}
	}

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

}

/* ========================================== *\
 * 		4. DESCRIPTOR MATCHING
\* ========================================== */

void PCAligner::matchSHOTDescriptors (){

	pcl::KdTreeFLANN<pcl::SHOT352> matching;
	matching.setInputCloud(shot_descriptors1_);

	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());

	std::cout << "--- Matching SHOT descriptors..." << std::endl;

	// Check every descriptor computed for the filteredCloud2.
	for (size_t i = 0; i < shot_descriptors2_->size(); ++i)
	{
		std::vector<int> neighbors(1);
		std::vector<float> squaredDistances(1);
		// Ignore NaNs.
		if (pcl_isfinite(shot_descriptors2_->at(i).descriptor[0]))
		{
			// find the nearest neighbor
			int neighborCount = matching.nearestKSearch(shot_descriptors2_->at(i), 1, neighbors, squaredDistances);
			if (neighborCount == 1 && squaredDistances[0] < 0.25f)
			{
				pcl::Correspondence correspondence(neighbors[0], static_cast<int>(i), squaredDistances[0]);
				correspondences->push_back(correspondence);
			}
		}
	}

	std::cout << "--- Descriptor matching complete. Found " << correspondences->size() << " correspondences." << std::endl;
}

/* ========================================== *\
 * 		5. SAMPLE CONSENSUS
\* ========================================== */

int PCAligner::SampleConsensus(){

	PCXYZ::Ptr tmp_aligned_cloud1 = PCXYZ::Ptr (new PCXYZ);

	pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::SHOT352> ransac;

	ransac.setInputSource(filtered_estimation_cloud1_);	// calculate transformation from alignment of filtered cloud 1 to filtered cloud 2
	ransac.setSourceFeatures(shot_descriptors1_);
	ransac.setInputTarget(filtered_estimation_cloud2_);
	ransac.setTargetFeatures(shot_descriptors2_);

	ransac.setCorrespondenceRandomness(2);
	// Inlier fraction to accept transform
	ransac.setInlierFraction(0.25f);
	// 6 DOF = 3
	ransac.setNumberOfSamples(3);
	ransac.setSimilarityThreshold(0.6f);
	ransac.setMaxCorrespondenceDistance(0.2f);

	std::cout << "--- Performing RANSAC approximation..." << std::endl;
	ransac.align(*tmp_aligned_cloud1); // BEFORE ransac.hasConverged() !

	if (ransac.hasConverged())
	{
		ransac_transformation_ = ransac.getFinalTransformation();
		std::cout << "Transformation matrix:" << std::endl << std::endl;
		printRotTrans(ransac_transformation_);
	}
	else{
		std::cout << "--- RANSAC did not converge." << std::endl;
		return -1;
	}

	// apply transformation on original cloud (not the downsampled one). Needs to be done for refinement
	ransac_aligned_cloud1_ = PCXYZ::Ptr (new PCXYZ);
	pcl::transformPointCloud (*cloud1_, *ransac_aligned_cloud1_, ransac_transformation_);

	// visualize sub result
	if(display_sub_results_)
	{

		pcl::visualization::PCLVisualizer viewer_ ("Prealignement using SHOT feature matching & RANSAC");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud1_, 0, 0, 255);
		viewer_.addPointCloud (cloud2_, source_cloud_color_handler, "original cloud");

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (ransac_aligned_cloud1_, 230, 20, 20);
		viewer_.addPointCloud (ransac_aligned_cloud1_, transformed_cloud_color_handler, "RANSAC aligned cloud");

		viewer_.addCoordinateSystem (1.0);

		while (!viewer_.wasStopped ()) { // Display the visualiser until 'q' key is pressed
			viewer_.spinOnce ();
		}
	}

	return 0;

}

/* ========================================== *\
 * 		6. ICP REFINEMENT
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

	std::cout << "--- Performing ICP refinement..." << std::endl;

	icp_aligned_cloud1_ = PCXYZ::Ptr (new PCXYZ);
	icp.align(*icp_aligned_cloud1_); // BEFORE pose.hasConverged() !


	std::cout << "\nhas converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;

	if (icp.hasConverged())
	{
		icp_transformation_ = icp.getFinalTransformation ();

		std::cout << "ICP refinement transformation:" << std::endl << std::endl;
		printRotTrans(icp_transformation_);
		std::cout << "Combined transformation:" << std::endl << std::endl;
		printRotTrans(getFinalTransformation());
	}
	else{
		std::cout << "ICP Did not converge." << std::endl;
		return -1;
	}

	// apply both transformation to the original cloud
	if(display_end_result_)
	{
		std::cout << "--- Preparing end result visualization..." << std::endl;

		PCXYZ::Ptr end_result = PCXYZ::Ptr (new PCXYZ);
		pcl::transformPointCloud (*cloud1_, *end_result, getFinalTransformation());		// transform cloud 1 to the frame of cloud2

		// visualize
		pcl::visualization::PCLVisualizer viewer2_ ("END RESULT");

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (end_result, 0, 0, 255);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (cloud2_, 230, 20, 20);

		viewer2_.addPointCloud (cloud2_, source_cloud_color_handler, "cloud2");
		viewer2_.addPointCloud (end_result, transformed_cloud_color_handler, "transformed cloud1");
		viewer2_.addCoordinateSystem (1.0);

		while (!viewer2_.wasStopped ()) {
			viewer2_.spinOnce ();
		}
	}



	return 0;

}

/* ========================================== *\
 * 		DATA EXTRACTION
\* ========================================== */

Eigen::Matrix4f PCAligner::getFinalTransformation(){
	// P1_in2 = H2_to1.1*H1.1_to1*P1_in1
	// cloud concatenation: cloud_combined = cloud2_ + finalTransformation*cloud1_
	return icp_transformation_*ransac_transformation_;

}

void PCAligner::printRotTrans(Eigen::Matrix4f mat){

	Eigen::Matrix3f rotation = mat.block<3, 3>(0, 0);
	Eigen::Vector3f translation = mat.block<3, 1>(0, 3);

	std::cout << std::endl;
	printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
	printf("\t\tR = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
	printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
	std::cout << std::endl;
	printf("\t\tt = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
	std::cout << std::endl;

}

