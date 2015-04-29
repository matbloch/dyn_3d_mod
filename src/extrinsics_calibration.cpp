#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <atomic>

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

// Boost
#include <boost/thread/thread.hpp>

#include "definitions.h"
#include "pcl/pc_aligner.h"
#include "config/config_handler.h"
#include "misc/colio.h"

using namespace sensor_msgs;
using namespace message_filters;

// status
std::atomic<bool> start_calibration(false);
std::atomic<bool> in_calibration(false);
std::atomic<bool> calib_is_finished(false);
std::atomic<bool> calib_cycle_finished(false);
std::atomic<bool> camera_is_connected(false);
std::atomic<bool> camera_timed_out(false);

// settings
ConfigHandler conf;
PCAligner aligner;

// alignement parameters
float cutoff_dist;
float approx_leaf_size;
float normal_est_search_radius;
float icp_max_correlation_dist;
float shot_search_radius;

void print_instructions();
void print_save_quit_instructions();
void ros_thread(unsigned int,unsigned int);	// includes the ros spinner
void interface_thread();  // user interface

void calibration_callback(
		const sensor_msgs::PointCloud2ConstPtr& c1, const sensor_msgs::PointCloud2ConstPtr& c2)
{

	if(calib_is_finished){return;}									// handle to trigger node termination
	if(!camera_is_connected){camera_is_connected = true;}			// camera status
	if(!start_calibration){return;}else{start_calibration = false;}	// handle to trigger calibration cylce

	in_calibration = true;	// status

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);

    // Convert point clouds to PCL format
	pcl::fromROSMsg(*c1, *cloud1);
	pcl::fromROSMsg(*c2, *cloud2);

	/* ========================================== *\
	 * 		DO ALIGNMENT
	\* ========================================== */

	aligner.setInputClouds(cloud1, cloud2);
	aligner.startAlignment();

	calib_cycle_finished = true;

	// wait to restart/quit
	while(calib_cycle_finished){
		usleep(500);
	}

	// stop callback till calibration is initiated again
	in_calibration = false;


}

int main(int argc, char** argv)
{
	/*
	 * @param optional, camera name 1
	 * @param optional, camera name 2
	 *
	 */

	ros::init(argc, argv, "dyn_3d_photo");
	ros::NodeHandle nh_;
	ros::Rate r(60); // 60 Hz
	image_transport::ImageTransport it_(nh_);

	std::string camera1;
	std::string camera2;

	// resolve topics
	if(argc > 2){
		camera1 = nh_.resolveName(argv[2]);
		camera2 = nh_.resolveName(argv[3]);
	}else{
		camera1 = nh_.resolveName("camera1");
		camera2 = nh_.resolveName("camera2");
	}

    std::string point_topic_1 = ros::names::clean(camera1 + "/depth/" + nh_.resolveName("points"));
    std::string point_topic_2 = ros::names::clean(camera2 + "/depth/" + nh_.resolveName("points"));

	// define subscribers
	message_filters::Subscriber<sensor_msgs::PointCloud2> depth_sub_1(nh_, point_topic_1, 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> depth_sub_2(nh_, point_topic_2, 1);

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
															sensor_msgs::PointCloud2>
	ApproximateSyncPolicy;

	message_filters::Synchronizer<ApproximateSyncPolicy> sync(ApproximateSyncPolicy(10),
			depth_sub_1
			,depth_sub_2
			);
	  sync.registerCallback(boost::bind(&calibration_callback,
			  _1
			  ,_2
			  ));

	ROS_INFO("Configuration node initialized");

	// setup aligner
	conf.getOption("extrinsics_calibration.approx_leaf_size",approx_leaf_size);
	conf.getOption("extrinsics_calibration.normal_est_search_radius",normal_est_search_radius);
	conf.getOption("extrinsics_calibration.icp_max_correlation_dist",icp_max_correlation_dist);
	conf.getOption("extrinsics_calibration.shot_search_radius",shot_search_radius);
	conf.getOption("extrinsics_calibration.cutoff_dist",cutoff_dist);
	aligner.setLeafSize(approx_leaf_size);
	aligner.setNormalEstSearchRadius(normal_est_search_radius);
	aligner.setICPMaximumCorrelationDist(icp_max_correlation_dist);
	aligner.setSHOTSearchRadius(shot_search_radius);
	aligner.setCutoffDistance(cutoff_dist);

	// start threads
    boost::thread_group threads;
    boost::thread *t1 = new boost::thread(ros_thread, 30, 5);	// @ 30 Hz and 5 sec connection timeout
    boost::thread *t2 = new boost::thread(interface_thread);
    threads.add_thread(t1);
    threads.add_thread(t2);

    threads.join_all();

	return 0;
}



/* ========================================== *\
 * 		UI
\* ========================================== */

void print_instructions(){

	std::cout<<"\n";
	std::cout << "=================================" << std::endl;
	std::cout << " INSTRUCTION:" << std::endl;
	std::cout << "---------------------------------" << std::endl;
	std::cout << " [space]: start calibration" << std::endl;
	std::cout << " [q]: press q in visualization windows to continue to next step" << std::endl;
	std::cout << "=================================" << std::endl;
	std::cout<<"\n";

}

void print_save_quit_instructions(){

	std::cout<<"\n";
	std::cout << "=================================" << std::endl;
	std::cout << " HOW WOULD YOU LIKE TO PROCEDE?:" << std::endl;
	std::cout << "---------------------------------" << std::endl;
	std::cout << " [s]: save extrinsic" << std::endl;
	std::cout << " [q]: quit configuration" << std::endl;
	std::cout << " [r]: restart configuration" << std::endl;
	std::cout << "=================================" << std::endl;
	std::cout<<"\n";

	char k2;
	k2 = 'c';
	while(k2 != 's' && k2 != 'q' && k2 != 'r'){

		k2 = getch();

		std::cout << k2 << std::endl;

	    if (k2 == 's'){

			Eigen::Matrix4f transf;

			transf = aligner.getFinalTransformation();

			// store extrinsics
			if(conf.updateOptionMatrix("camera_parameters.extrinsics", transf)){
				calib_is_finished = true;
			}else{
				std::cout << "An error occurred during saving the extrinsic. Please make sure to include the following path in your configuration file: camera_parameters.extrinsics" << std::endl;
			}

	    }else if(k2 == 'q'){
	        	calib_is_finished = true;
	    }else if(k2 == 'r'){
	    	// print instructions
			print_instructions();
	    }else{
	    	std::cout << "Please select a valid option." << std::endl;
	    }
	}

	calib_cycle_finished = false;
}

void ros_thread(unsigned int rate = 30, unsigned int  t = 5){


	std::cout <<  "--- Waiting for camera connection..." << std::endl;

	ros::Rate r(rate); // 30 Hz

	unsigned int timeout = t;  // connection timeout in seconds
	time_t init_time = time(0);

	while (!calib_is_finished)	// spin until calibration has ended
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

	// wait till camera is connected
	while(!camera_is_connected){
		if(camera_timed_out){return;}
		usleep(300);
	}

	std::cout << GREEN << "--- Camera connection established. Ready to do calibration." << RESET << endl;

	print_instructions();

    char k;

	while(!calib_is_finished){

		// right after calibration: save
		if(!in_calibration){
			k = getch();
			if (k == ' '){
				start_calibration = true;
				in_calibration = true;	// needs to be triggered already here (ros spinner delay)
			}
		}else if(calib_cycle_finished){
			// in calibration
			print_save_quit_instructions();

		}

		usleep(300);	// interface update rate
	}

}
