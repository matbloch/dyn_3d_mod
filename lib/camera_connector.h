#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "definitions.h"	// constants
 
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


using namespace std;

class CameraConnector
{

/*
 * @prop cv_ptr: opencv mat
 * @prop pc_ptr: PointCloud2
 * @prop running: bool
 *
 */
	public:

		ros::NodeHandle nh_;	// started on ros::init
		image_transport::ImageTransport it_;

		// depth map
		cv_bridge::CvImagePtr cv_ptr;   // access the image by cv_ptr->image
		pcl::PCLPointCloud2Ptr pc_ptr; 	// access point cloud


		// camera status
		bool running;

	CameraConnector(): it_(nh_)
	{
	  running = false;

	  // Create depth image subscriber
	  image_transport::Subscriber image_sub_ = it_.subscribe("/camera/depth/image", 1, &CameraConnector::depth_callback, this);

	  // Create a ROS subscriber for the input point cloud
	  ros::Subscriber sub = nh_.subscribe ("/camera/depth/points", 1, &CameraConnector::pc_callback, this);


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

	void pc_callback (const pcl::PCLPointCloud2ConstPtr& cloud)
	{
	/*
	 * TODO: visualize pc using pcl visualizer
	 *
	 */



	}

	bool save_depth_image()
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
