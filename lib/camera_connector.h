#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "definitions.h"	// constants
 
using namespace std;

class CameraConnector
{

/*
 * @prop cv_ptr: opencv mat
 * @prop running: bool
 *
 */
	public:

		ros::NodeHandle nh_;	// started on ros::init
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;

		// depth map
		cv_bridge::CvImagePtr cv_ptr;   // access the image by cv_ptr->image

		// camera status
		bool running;

	CameraConnector(): it_(nh_)
	{
	  running = false;
	  image_sub_ = it_.subscribe("/camera/depth/image", 1, &CameraConnector::depth_callback, this);
	}
	~CameraConnector()
	{

	}

	void depth_callback(const sensor_msgs::ImageConstPtr& msg)
	{
	/*
	* converts the incoming ROS message to an OpenCV image
	*
	*/

		try{
		// convert to cv: 32bit float representing meters (rostopic interprets as int8)
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);

			// image received - update status
			if(!running){
				std::cout << GREEN << "--- Camera connection established" << RESET << endl;
				running = true;
			}

		}catch (cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
	}

	bool save_depth()
	{
	/*
	 * Saves a snapshot of the depth stream as a .png image
	 * TODO: add datetime to filename
	 */
	  std::string path = ros::package::getPath("dyn_3d_mod");
	  path.append("/recordings/images/depth_capture_.png");

	  return cv::imwrite(path, cv_ptr->image);
	}

};
