#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
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
bool isfirst = true;

// grid status
static int MAX_DIM = 7;
static int GRID_SIZE = (int)pow(2,MAX_DIM);
static float spacing_in_m = 0.02;
float max_v[3] = {(float)(GRID_SIZE-1), (float)(GRID_SIZE-1), (float)(GRID_SIZE-1)};
float min_v[3] = {0, 0, 0};

TStree::TStree tstree(max_v[0]-min_v[0], max_v, min_v, MAX_DIM);

// settings
ConfigHandler conf;
ImageFilters filters;
voxelGrid grid;

// globals
ros::Publisher pub_;

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
     * 		1. Frame transformation
    \* ========================================== */

    /*
     * cv_ptr1->image ...
     */

    cv::Mat filtered1;
    cv::Mat filtered2;

    // prefiltering
    filters.gaussian(cv_ptr1, filtered1);
    filters.gaussian(cv_ptr1, filtered2);

    // display filter result
	cv::imshow("first image", filtered1);
	cv::waitKey(3);
	cv::imshow("second image", filtered2);
	cv::waitKey(3);

    /* ========================================== *\
     * 		2. Voxel grid
    \* ========================================== */

    // get extrinsics
    Eigen::Matrix4f extrinsics;
	conf.getOptionMatrix("camera_parameters.extrinsics", extrinsics);

	// Define intrinsic camera parameters
	cv::Mat intrinsicMat(3, 3, CV_32F); // intrinsic matrix
	getCameraParameters(intrinsicMat);

	// Needs to be edited
	// Define Camera orientation and position w.r.t. grid
	cv::Mat R(3, 3, CV_32F);
	cv::Mat tVec(3, 1, CV_32F); // Translation vector in camera frame
	getCameraPose(R,tVec);

	// Setup voxel structure to load the TSDF in
	int sz[3] = {GRID_SIZE,GRID_SIZE,GRID_SIZE};
	Mat FilledVoxels(3,sz, CV_32FC1, Scalar::all(0));

	// Setup the grid
	grid.setParameters(GRID_SIZE, spacing_in_m, intrinsicMat, R, tVec);
	// Fill in voxels
	grid.fillVoxels(cv_ptr1>image, FilledVoxels);


    /* ========================================== *\
     * 		3. Fusion
    \* ========================================== */

  //get grid range
  for(int i=0; i<3; i++){
    max_v[i] = (grid.units).at(GRID_SIZE-1);
    min_v[i] = (grid.units).at(0);
  }

  //initialize tstree
  if(isfirst){
    tstree.setGridRange(max_v[0]-min_v[0], max_v, min_v;
  }

  //reset nan as 1
  for(int i=0; i<GRID_SIZE; i++){
    for(int j=0; j<GRID_SIZE; j++){
      for(int k=0; k<GRID_SIZE; k++){
        if(isnan(FilledVoxels.at<float>(i,j,k))){
          FilledVoxels.at<float>(i,j,k) = 1;
        }
      }
    }
  }

  tstree.insert(FilledVoxels, 0);

  //grid_values = tstree.read(0);
    /* ========================================== *\
     * 		4. Publishing
    \* ========================================== */

	// convert back to ROS message
    pcl::toROSMsg(output_pcl, output);

    // publish the concatenated point cloud
    pub_.publish(output);

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
	message_filters::Subscriber<sensor_msgs::Image> depth_sub_1(nh, "/camera1/depth/image", 1);
	message_filters::Subscriber<sensor_msgs::Image> depth_sub_2(nh, "/camera2/depth/image", 1);

	// callback if message with same timestaps are received (buffer length: 10)
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
	                                                        sensor_msgs::Image>
	SyncPolicy;

	message_filters::Synchronizer< SyncPolicy > sync(SyncPolicy(10), depth_sub_1, depth_sub_2);

	// add callback
	sync.registerCallback(boost::bind(&preprocessing_callback, _1, _2));

	// register publisher for the merged clouds
	pub_ = nh.advertise<sensor_msgs::Image>(ros::this_node::getName() + "/preprocessed_data", 1);

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
