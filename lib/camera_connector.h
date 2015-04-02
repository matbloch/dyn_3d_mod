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
#include <pcl_ros/point_cloud.h>	// TODO: not sure if to use pcl/point_cloud or pcl_ros/point_cloud


using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

class CameraConnector
{

/*
 * @prop cv_ptr: opencv mat
 * @prop running: bool
 *
 */
	private:

		ros::NodeHandle nh_;	// started on ros::init
		image_transport::ImageTransport it_;

		// point cloud
		ros::Subscriber pc_sub_;

		// registered point cloud
		ros::Subscriber pc_reg_sub_;

		 // PCL visualization
		pcl::visualization::CloudViewer viewer_;
		//boost::shared_ptr<pcl::visualization::CloudViewer> viewer_;


	public:

		// depth map
		cv_bridge::CvImagePtr cv_ptr;   // access the image by cv_ptr->image

		// status
		bool running;
		bool pc_running;
		bool pc_reg_running;
		bool show_depth_image;
		bool show_pc;
		bool show_pc_reg;

	CameraConnector(bool flag);

	~CameraConnector();

	/* message callbacks */

	void depth_callback(const sensor_msgs::ImageConstPtr& msg);
	void pc_callback ( const sensor_msgs::PointCloud2ConstPtr& msg );
	void pc_registered_callback ( const sensor_msgs::PointCloud2ConstPtr& msg );

	/* storage function */
	bool save_depth_image();

	void createViewer();


	/* visualization sub class */
    class Visualizer
    {

    	CameraConnector& parent_;	// CameraConnector
    	public:
    		Visualizer(CameraConnector& parent);
    		void depth_image();
    		void pc();
    		void pc_reg();
    };

    /* accessible through: {CameraConnector instance}.show */
    Visualizer show;


};

/* ========================================== *\
 * 		CONSTRUCTOR/DESTRUCTOR
\* ========================================== */

/* constructor */
CameraConnector::CameraConnector(bool flag):
	it_(nh_),
	nh_("~"),
	viewer_("Point Cloud Viewer"),
	show(*this)
{

  // init status
  running = false;
  pc_running = false;
  pc_reg_running = false;
  show_depth_image = false;
  show_pc = false;
  show_pc_reg = false;

  // Create depth image subscriber
  image_transport::Subscriber image_sub_ = it_.subscribe("/camera/depth/image", 1, &CameraConnector::depth_callback, this);

  // Subscribe to depth point cloud
  pc_sub_ = nh_.subscribe ("/camera/depth/points", 1, &CameraConnector::pc_callback, this);

  // Subscribe to rgb point cloud (registered)
 // pc_reg_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &CameraConnector::pc_registered_callback, this);

  ROS_INFO("Waiting to receive camera stream...");

}

CameraConnector::~CameraConnector(){}

/* ========================================== *\
 * 		MESSAGE CALLBACKS
\* ========================================== */

void CameraConnector::depth_callback(const sensor_msgs::ImageConstPtr& msg)
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


		if(show_depth_image){
			cv::imshow(OPENCV_WINDOW, cv_ptr->image);
			cv::waitKey(3);
		}

	}catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

void CameraConnector::pc_callback ( const sensor_msgs::PointCloud2ConstPtr& msg )
{

	// image received - update status
	if(!pc_running){
		std::cout << GREEN << "--- Receiving Point Cloud stream" << RESET << endl;
		pc_running = true;
	}

	if(show_pc){
		 // convert to XYZ PC
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*msg, *pc);

		// visualize
		/*
		if (!viewer_->wasStopped()){
			viewer_->showCloud(pc);		// use boost pointer
		}
		*/
		//viewer_.showCloud(pc);
	}

}

void CameraConnector::pc_registered_callback ( const sensor_msgs::PointCloud2ConstPtr& msg )
{

	// image received - update status
	if(!pc_reg_running){
		std::cout << GREEN << "--- Receiving Point Cloud stream" << RESET << endl;
		pc_reg_running = true;
	}

	if(show_pc_reg){
		 // convert to XYZ PC
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr rpc(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::fromROSMsg(*msg, *rpc);

		// visualize
		//viewer_.showCloud(rpc);
	}

}

// Creates, initializes and returns a new viewer.
void CameraConnector::createViewer()
{

	//viewer_ = (new pcl::visualization::CloudViewer("OpenNI viewer"));
	//boost::shared_ptr<visualization::CloudViewer> v (new visualization::CloudViewer("OpenNI viewer"));

}

/* ========================================== *\
 * 		VISUALIZATION
\* ========================================== */

CameraConnector::Visualizer::Visualizer(CameraConnector& parent) : parent_(parent) {


}

void CameraConnector::Visualizer::depth_image()
{
	parent_.show_depth_image = true;
}
void CameraConnector::Visualizer::pc()
{
	parent_.show_pc = true;
}
void CameraConnector::Visualizer::pc_reg()
{
	parent_.show_pc_reg = true;
}

/* ========================================== *\
 * 		DATA EXTRACTION
\* ========================================== */

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




