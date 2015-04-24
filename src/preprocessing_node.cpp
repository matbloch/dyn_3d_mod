#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// OpenCV includes
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

// Message filters
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/sync_policies/exact_time.h>		// exact time synchronization
#include <message_filters/sync_policies/approximate_time.h>	// approximate time synchronization

// Eigen
#include <Eigen/Dense>

// our own libraries
#include <config/config_handler.h>
#include <cv/filters.h>
#include "cv/voxel_grid.h"

// status
bool in_calibration = false;
bool calib_is_finished = false;
bool camera_is_connected = false;

// object handlers
ConfigHandler conf;
ImageFilters filters;
voxelGrid grid1;
voxelGrid grid2;

// globals
ros::Publisher pub_;
cv::Mat intrinsicMat(3, 3, CV_32F); // intrinsic matrix
cv::Mat R1(3, 3, CV_32F);    // Rotation vector
cv::Mat tVec1(3, 1, CV_32F); // Translation vector in camera frame
cv::Mat R2(3, 3, CV_32F);    // Rotation vector
cv::Mat tVec2(3, 1, CV_32F); // Translation vector in camera frame
int gridsize;   // Must be 2^x
float spacing_in_m;
cv::Mat FilledVoxels1;
cv::Mat FilledVoxels2;
cv::Mat FusedVoxels;


void getCameraParameters(cv::Mat intrinsicMat);
void getCameraPose(cv::Mat R, cv::Mat tVec);

void preprocessing_callback(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2)
{

	/*
	 * TODO:
	 * 1. frame transformation
	 * 2. Voxel grid
	 * 3. Fusion
	 * 4. Publish as custom message
	 */


	ROS_INFO("Synced callback was called.");

    /* ========================================== *\
     * 		0. OpenCV conversion
    \* ========================================== */

	cv_bridge::CvImagePtr cv_ptr1;
	cv_bridge::CvImagePtr cv_ptr2;

    try
    {
      cv_ptr1 = cv_bridge::toCvCopy(msg1, sensor_msgs::image_encodings::TYPE_32FC1);
      cv_ptr2 = cv_bridge::toCvCopy(msg2, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    /* ========================================== *\
     * 		1. Filtering
    \* ========================================== */

    cv::Mat filtered1;
    cv::Mat filtered2;

    // No filtering
    filtered1 = cv_ptr1->image;
    filtered2 = cv_ptr2->image;

    // prefiltering
//    filters.bilateral(cv_ptr1->image, filtered1);
//    filters.bilateral(cv_ptr2->image, filtered2);


    /*
     * store depth data as .yml
    cv::FileStorage storage("test_data.yml", cv::FileStorage::WRITE);
    storage << "img" << filtered1;
    storage.release();


    */

//	cv::imshow("filtered", filtered1);
//	cv::waitKey(30000);

    /* ========================================== *\
     * 		2. Voxel grid
    \* ========================================== */


    // get extrinsics
//    Eigen::Matrix4f extrinsics;
//	conf.getOptionMatrix("camera_parameters.extrinsics", extrinsics);

	grid1.fillVoxels(filtered1, FilledVoxels1);
	grid2.fillVoxels(filtered2, FilledVoxels2);

	ROS_INFO("Voxels filled");

    /* ========================================== *\
     * 		3. Fusion
    \* ========================================== */

	FusedVoxels = 0.5*(FilledVoxels1 + FilledVoxels2);

    /* ========================================== *\
     * 		4. Publishing
    \* ========================================== */

	// convert back to ROS message
    //pcl::toROSMsg(output_pcl, output);

    // publish the concatenated point cloud
    //pub_.publish(output);

}


int main(int argc, char** argv)
{
	/*
	 * INPUT: camera streams (sensor image msg)
	 * OUTPUT: synced, aligned voxel structure (CV::Mat)
	 *
	 */

	ros::init(argc, argv, "dyn_3d_photo");
	ros::NodeHandle nh;

	// define subscribers
	message_filters::Subscriber<sensor_msgs::Image> depth_sub_1(nh, "/camera1/depth/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::Image> depth_sub_2(nh, "/camera2/depth/image_raw", 1);

	// callback if message with same timestaps are received (buffer length: 10)
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
	                                                        sensor_msgs::Image>
	SyncPolicy;

	message_filters::Synchronizer< SyncPolicy > sync(SyncPolicy(10), depth_sub_1, depth_sub_2);

	// add callback
	sync.registerCallback(boost::bind(&preprocessing_callback, _1, _2));

	// register publisher for the merged clouds
	pub_ = nh.advertise<sensor_msgs::Image>(ros::this_node::getName() + "/preprocessed_data", 1);

	// Set up the voxel grid objects for both cameras
	getCameraParameters(intrinsicMat);
	getCameraPose(R1,tVec1);
	getCameraPose(R2,tVec2);
	gridsize = 64;   // Must be 2^x
	spacing_in_m = 0.05;
	grid1.setParameters(gridsize, spacing_in_m, intrinsicMat, R1, tVec1);
	grid2.setParameters(gridsize, spacing_in_m, intrinsicMat, R2, tVec2);

	// Setup voxel structure to load the TSDF in
	int sz[3] = {gridsize,gridsize,gridsize};
	FilledVoxels1 = Mat(3,sz, CV_32FC1, Scalar::all(0));
	FilledVoxels2 = Mat(3,sz, CV_32FC1, Scalar::all(0));
	FusedVoxels = Mat(3,sz, CV_32FC1, Scalar::all(0));

	ROS_INFO("Preprocessing node initalized.");

	while(ros::ok()){
	  	ros::spinOnce();
	}

	return 0;
}



void getCameraParameters(cv::Mat intrinsicMat)
{
	intrinsicMat.at<float>(0, 0) = 589.3667;  // 640/2/tand(57/2)
	intrinsicMat.at<float>(1, 0) = 0;
	intrinsicMat.at<float>(2, 0) = 0;

	intrinsicMat.at<float>(0, 1) = 0;
	intrinsicMat.at<float>(1, 1) = 609.2755;  // 480/2/tand(43/2)
	intrinsicMat.at<float>(2, 1) = 0;

	intrinsicMat.at<float>(0, 2) = 319.5;  // 640/2
	intrinsicMat.at<float>(1, 2) = 239.5;  // 240/2
	intrinsicMat.at<float>(2, 2) = 1;
}

void getCameraPose(cv::Mat R, cv::Mat tVec)
{
	cv::Mat rVec(3, 1, cv::DataType<float>::type); // Rotation vector
	// Rotation vector in Rodrigues angles
	rVec.at<float>(0) = 0;
	rVec.at<float>(1) = 0;
	rVec.at<float>(2) = 0;
	Rodrigues(rVec, R);

	// Translation vector in Camera frame
	tVec.at<float>(0) = 0;
	tVec.at<float>(1) = 0;
	tVec.at<float>(2) = -5;
}
