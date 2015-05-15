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

// status
std::atomic<bool> recording(false);
std::atomic<bool> recording_finished(false);
std::atomic<bool> camera_is_connected(false);
std::atomic<bool> camera_timed_out(false);

using namespace cv;
using namespace std;

void print_instructions();
void ros_thread(unsigned int, unsigned int );
void interface_thread();

void callback(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2)
{

	// update connection status
	if(!camera_is_connected){camera_is_connected=true;}

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

	cv::imshow("Camera 1", cv_ptr1->image);
	cv::waitKey(3);

	cv::imshow("Camera 2", cv_ptr2->image);
	cv::waitKey(3);

	Mat combine(max(cv_ptr1->image.size().height, cv_ptr2->image.size().height), cv_ptr1->image.size().width + cv_ptr2->image.size().width, CV_8UC3);


	cv::imshow("Depth images", combine);
	cv::waitKey(3);


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
	sync.registerCallback(boost::bind(&callback, _1, _2));

	ROS_INFO("Snapshot node initalized.");
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


/* ========================================== *\
 * 		UI
\* ========================================== */

void print_instructions(){

	std::cout<<"\n";
	std::cout << "=================================" << std::endl;
	std::cout << " INSTRUCTION:" << std::endl;
	std::cout << "---------------------------------" << std::endl;
	std::cout << " [space]: take a snapshot of the depth images" << std::endl;
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

		// start recording on [space]
			while(true){
				k = getch();

				if(k == ' '){
					if(!recording){
						recording = true;
					}
				}
			}
	}

	return;

}
