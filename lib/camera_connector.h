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

#include <pcl/visualization/cloud_viewer.h>

#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>


using namespace std;

class CameraConnector
{

/*
 * @prop cv_ptr: opencv mat
 * @prop running: bool
 *
 */
	private:


	public:

		ros::NodeHandle nh_;	// started on ros::init
		image_transport::ImageTransport it_;

		// depth map
		cv_bridge::CvImagePtr cv_ptr;   // access the image by cv_ptr->image

		// point cloud
		ros::Subscriber pc_sub_;

		// registered point cloud
		ros::Subscriber pc_reg_sub_;

		 // PCL visualizer
		pcl::visualization::CloudViewer viewer_;


	public:

		// camera status
		bool running;
		bool pc_running;
		bool pc_reg_running;

	CameraConnector(bool flag):
		it_(nh_),
		nh_("~"),
		viewer_("Simple Cloud Viewer")
	{
	  // init status
	  running = false;
	  pc_running = false;
	  pc_reg_running = false;

	  // Create depth image subscriber
	  image_transport::Subscriber image_sub_ = it_.subscribe("/camera/depth/image", 1, &CameraConnector::depth_callback, this);

	  // Subscribe to depth point cloud
	  pc_sub_ = nh_.subscribe ("/camera/depth/points", 1, &CameraConnector::pc_callback, this);

	  // Subscribe to rgb point cloud (registered)
	 // pc_reg_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &CameraConnector::pc_registered_callback, this);

	  ROS_INFO("Waiting to receive camera stream...");

	}
	~CameraConnector()
	{

	}

	void depth_callback(const sensor_msgs::ImageConstPtr& msg)
	{
		/*
		* converts the incoming ROS message to an OpenCV image
		*/

		try{
		// convert to cv: 32bit float representing meters (rostopic interprets as int8)
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);

			// image received - update status
			if(!running){
				std::cout << GREEN << "--- Receiving depth image stream" << RESET << endl;
				running = true;
			}

		}catch (cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
	}

	void pc_callback ( const sensor_msgs::PointCloud2ConstPtr& msg )
	{

		// image received - update status
		if(!pc_running){
			std::cout << GREEN << "--- Receiving Point Cloud stream" << RESET << endl;
			pc_running = true;
		}

		 // convert to XYZ PC
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*msg, *pc);

		// visualize
		viewer_.showCloud(pc);


	}

	void pc_registered_callback ( const sensor_msgs::PointCloud2ConstPtr& msg )
	{

		// image received - update status
		if(!pc_reg_running){
			std::cout << GREEN << "--- Receiving Point Cloud stream" << RESET << endl;
			pc_reg_running = true;
		}

		 // convert to XYZ PC
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr rpc(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::fromROSMsg(*msg, *rpc);

		// visualize
		viewer_.showCloud(rpc);

	}


	bool save_depth_image()
	{
	/*
	 * Saves a snapshot of the depth stream as a .png image
	 * TODO: add datetime to filename
	 */
	  std::string path = ros::package::getPath("dyn_3d_mod");
	  //path.append("/recordings/images/depth_capture_.png");

	  //return cv::imwrite(path, cv_ptr->image);
	  return 0;
	}

};
