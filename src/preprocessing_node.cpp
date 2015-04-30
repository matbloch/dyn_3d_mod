#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <iostream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <atomic>

// Eigen
#include <Eigen/Dense>

// OpenCV includes
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

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

// Boost
#include <boost/thread/thread.hpp>

// our own libraries
#include <config/config_handler.h>
#include <cv/filters.h>
#include "cv/voxel_grid.h"
#include "misc/colio.h"
#include "definitions.h"

// time space tree library
#include "tree/TStree.hpp"

// object handlers
ConfigHandler conf;
ImageFilters filters;
voxelGrid grid1;
voxelGrid grid2;

// globals
cv::Mat intrinsicMat(3, 3, CV_32F); // intrinsic matrix
cv::Mat R1(3, 3, CV_32F);    // Rotation vector
cv::Mat tVec1(3, 1, CV_32F); // Translation vector in camera frame
cv::Mat R2(3, 3, CV_32F);    // Rotation vector
cv::Mat tVec2(3, 1, CV_32F); // Translation vector in camera frame
cv::Mat FilledVoxels1;
cv::Mat FilledVoxels2;
cv::Mat FusedVoxels;

// status
std::atomic<bool> recording(false);
std::atomic<bool> recording_finished(false);
std::atomic<bool> camera_is_connected(false);
std::atomic<bool> camera_timed_out(false);

bool isfirst = true;

// grid status
static int MAX_DIM = 4;
static int GRID_SIZE = (int)pow(2,MAX_DIM);
static float spacing_in_m = 0.02;
float max_v[3] = {(float)(GRID_SIZE-1), (float)(GRID_SIZE-1), (float)(GRID_SIZE-1)};
float min_v[3] = {0, 0, 0};

// Time Space Tree
TStree tstree((GRID_SIZE-1)*spacing_in_m, MAX_DIM);
int t = 0;

void getCameraParameters(cv::Mat intrinsicMat);
void getCameraPose(cv::Mat R, cv::Mat tVec);
void print_instructions();

void ros_thread(unsigned int,unsigned int);	    // includes the ros spinner
void interface_thread();  // user interface


void preprocessing_callback(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2)
{

	//ROS_INFO("Synced callback was called.");

	// update connection status
	if(!camera_is_connected){camera_is_connected=true;}
	// check recording status
	if(!recording){return;}

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

	// prefiltering
	filters.bilateral(cv_ptr1->image, filtered1);
	filters.bilateral(cv_ptr2->image, filtered2);

	filters.replaceValue(filtered1, 0, 99);
	filters.replaceValue(filtered2, 0, 99);

	// to meters
	filtered1 = filtered1/1000.0;
	filtered2 = filtered2/1000.0;

	// set NaNs to 99 m (will be ignored in voxel conversion)
	//cv::patchNaNs(filtered1, 99);
	//cv::patchNaNs(filtered2, 99);

    /* ========================================== *\
     * 		2. Voxel grid
    \* ========================================== */


    // get extrinsics
//    Eigen::Matrix4f extrinsics;
//	conf.getOptionMatrix("camera_parameters.extrinsics", extrinsics);

	grid1.fillVoxels(filtered1, FilledVoxels1);
	grid2.fillVoxels(filtered2, FilledVoxels2);

	//ROS_INFO("Voxels filled");

    /* ========================================== *\
     * 		3. Fusion
    \* ========================================== */

	FusedVoxels = 0.5*(FilledVoxels1 + FilledVoxels2);

    /* ========================================== *\
     * 		4. Octree integration
    \* ========================================== */

  
  	tstree.insert(FilledVoxels1, t);
  	t++;
  	tstree.print_timespacetree();


}


int main(int argc, char** argv)
{

	ros::init(argc, argv, "dyn_3d_photo");
	ros::NodeHandle nh_;

	// define subscribers
	message_filters::Subscriber<sensor_msgs::Image> depth_sub_1(nh_, "/camera1/depth/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::Image> depth_sub_2(nh_, "/camera2/depth/image_raw", 1);

	// callback if message with same timestaps are received (buffer length: 10)
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
	                                                        sensor_msgs::Image>
	SyncPolicy;

	message_filters::Synchronizer< SyncPolicy > sync(SyncPolicy(10), depth_sub_1, depth_sub_2);

	// add callback
	sync.registerCallback(boost::bind(&preprocessing_callback, _1, _2));

	// Set up the voxel grid objects for both cameras
	getCameraPose(R1,tVec1);

	// get extrinsics
	Eigen::Matrix4f extrinsics;
	conf.getOptionMatrix("camera_parameters.extrinsics", extrinsics);
	cv::Mat extrinsics_mat(4,4, CV_32F);
	cv::eigen2cv(extrinsics, extrinsics_mat);
	cv::Mat R_ext = extrinsics_mat(cv::Range(0,3), cv::Range(0,3));
	cv::Mat t_ext = extrinsics_mat(cv::Range(0,3), cv::Range(3,4));

	// get intrinsics
	Eigen::Matrix3f intrinsics_eig;
	conf.getOptionMatrix("camera_parameters.intrinsics", intrinsics_eig);
	cv::eigen2cv(intrinsics_eig, intrinsicMat);

	// Set up the voxel grid objects for both cameras
	getCameraPose(R1,tVec1);
	R2 = R_ext*R1;
	tVec2 = t_ext + R_ext*tVec1;

	grid1.setParameters(GRID_SIZE, spacing_in_m, intrinsicMat, R1, tVec1);
	grid2.setParameters(GRID_SIZE, spacing_in_m, intrinsicMat, R2, tVec2);

	// Setup voxel structure to load the TSDF in
	int sz[3] = {GRID_SIZE,GRID_SIZE,GRID_SIZE};
	FilledVoxels1 = Mat(3,sz, CV_32FC1, Scalar::all(0));
	FilledVoxels2 = Mat(3,sz, CV_32FC1, Scalar::all(0));
	FusedVoxels = Mat(3,sz, CV_32FC1, Scalar::all(0));

	ROS_INFO("Preprocessing node initalized.");
	print_instructions();

	// start threads
    boost::thread_group threads;
    boost::thread *t1 = new boost::thread(ros_thread, 30, 5);	// @ 30 Hz and 5 sec connection timeout
    boost::thread *t2 = new boost::thread(interface_thread);
    threads.add_thread(t1);
    threads.add_thread(t2);

    threads.join_all();

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


/* ========================================== *\
 * 		UI
\* ========================================== */

void print_instructions(){

	std::cout<<"\n";
	std::cout << "=================================" << std::endl;
	std::cout << " INSTRUCTION:" << std::endl;
	std::cout << "---------------------------------" << std::endl;
	std::cout << " [space]: start/stop recording" << std::endl;
	std::cout << "=================================" << std::endl;
	std::cout<<"\n";

}

void ros_thread(unsigned int rate = 30, unsigned int  t = 5){

	ros::Rate r(rate); // standard: 30 Hz

	unsigned int timeout = t;  // connection timeout in seconds
	time_t init_time = time(0);

	while (!recording_finished)	// spin until calibration has ended
	{

		if(!camera_is_connected && time(0) > timeout + init_time){

			camera_timed_out = true;
			std::cout << RED << "--- Camera connection timed out." << RESET << endl;
			break;
		}

		ros::spinOnce();
		r.sleep();
	}

	return;


}
void interface_thread(){

	char k;

	// wait till camera is connected
	while(!camera_is_connected){
		if(camera_timed_out){return;}
		usleep(300);
	}

	std::cout << GREEN << "--- Camera connection established. Ready to record." << RESET << endl;

	// 1. RECORDING
	while(!recording_finished){

		// start > stop recording on [space]
		do{
			while(true){
				k = getch();

				if(k == ' '){
					if(recording){
						recording = false;
						std::cout << GREEN << "--- End recording..." << RESET << endl;
						// TODO: wait till buffer is processed
						break;
					}else{
						recording = true;
						std::cout << GREEN << "--- Start recording..." << RESET << endl;
						break;
					}
				}
			}

		}while(recording);

		std::cout<<"\n";
		std::cout << "=================================" << std::endl;
		std::cout << " HOW WOULD YOU LIKE TO CONINUE?" << std::endl;
		std::cout << "---------------------------------" << std::endl;
		std::cout << " [v]: visualize recorded scene" << std::endl;
		std::cout << " [r]: record new scene" << std::endl;
		std::cout << " [q]: quit" << std::endl;
		std::cout << "=================================" << std::endl;
		std::cout<<"\n";

		k = ' ';

		while(k != 'r' && k != 'v'){	// loop until valid selection
			k = getch();

			if(k == 'r'){
				// TODO: RESET THE OCTREE STRUCTURE AND BUFFER HERE

				std::cout << "--- Ready to record new scene." << RESET << endl;
			}else if(k == 'v'){
				// leave outer loop and continue
				recording = false;
				recording_finished = true;	// stops the ROS spinner

			}else if(k == 'q'){
				// leave outer loop and end interface loop
				recording_finished = true;	// stops the ROS spinner
				return;
			}else{
				std::cout << RED << "Please select a valid option" << RESET << std::endl;
			}
		}
	}

	// 1. VISUALIZATION
	int frame_nr;
	std::cout<<"\n";
	std::cout << "=================================" << std::endl;
	std::cout << "VISUALIZATION" << std::endl;
	std::cout << "---------------------------------" << std::endl;
	std::cout << "Specify the frame you want to visualize: ";
	std::cin >> frame_nr;




	return;

}
