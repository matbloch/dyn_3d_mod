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
#include <pcl/io/pcd_io.h>
//#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>	// TODO: not sure if to use pcl/point_cloud or pcl_ros/point_cloud

#include <pcl/filters/voxel_grid.h> // voxel grid

using namespace std;
using namespace pcl;


/* ========================================== *\
 * 		GLOBALS
\* ========================================== */
static const std::string OPENCV_WINDOW_COLOR = "Color stream";
static const std::string OPENCV_WINDOW_DEPTH = "Depth stream";
bool SAVE_CLOUD = false;

/* ========================================== *\
 * 		FORWARD DECLARATIONS
\* ========================================== */

void keyboardEventOccurred(const visualization::KeyboardEvent& event,void* nothing);
boost::shared_ptr<visualization::CloudViewer> createViewer();


class CameraConnector
{

/*
 * @prop cv_ptr->image: opencv mat
 * @prop running: bool
 *
 */

	private:

		ros::NodeHandle nh_;	// started on ros::init
		image_transport::ImageTransport it_;

		// point cloud subsscriber
		ros::Subscriber pc_sub_;

		// image subscribers
		image_transport::Subscriber image_color_sub_;
		image_transport::Subscriber image_depth_sub_;

	public:

		// depth & color images
		cv_bridge::CvImagePtr cv_ptr_depth;   // access the image by cv_ptr->image
		cv_bridge::CvImagePtr cv_ptr_color;   // access the image by cv_ptr->image

		// point cloud visualizer
		boost::shared_ptr<visualization::CloudViewer> pc_viewer_;

		// status
		bool running;
		bool pc_running;
		bool display_color_image;
		bool display_depth_image;
		bool display_cloud;


	CameraConnector(bool flag);	// the point cloud topic to subscribe to
	~CameraConnector();

	/* message callbacks */
	void depth_image_callback( const sensor_msgs::ImageConstPtr& msg );
	void color_image_callback( const sensor_msgs::ImageConstPtr& msg );
	void pc_callback ( const sensor_msgs::PointCloud2ConstPtr& msg );
	void pc_voxel_callback ( const pcl::PCLPointCloud2ConstPtr& msg);

	/* storage function */
	bool save_depth_image();

	/* display functions */
	void show_depth(bool show);
	void show_pc(bool show);
	void show_color(bool show);

	/* help */
	void display_help();

};

/* ========================================== *\
 * 		CONSTRUCTOR/DESTRUCTOR
\* ========================================== */

/* constructor */
CameraConnector::CameraConnector(
		bool flag):	// initialize non static data members with initialization list
	it_(nh_),	// image transform
	nh_("~")	// node handle
{

  // init status
  running = false;
  pc_running = false;
  display_color_image = false;
  display_cloud = false;

  SAVE_CLOUD = false;

  // Create image subscribers
  image_depth_sub_ = it_.subscribe("/camera/depth/image", 1, &CameraConnector::depth_image_callback, this);
  image_color_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &CameraConnector::color_image_callback, this);

  // Subscribe to depth point cloud
  pc_sub_ = nh_.subscribe("/camera/depth/points", 1, &CameraConnector::pc_callback, this);

  ROS_INFO("Waiting to receive camera stream...");
  display_help();

}

CameraConnector::~CameraConnector(){
	cv::destroyWindow(OPENCV_WINDOW_COLOR);
	cv::destroyWindow(OPENCV_WINDOW_DEPTH);
}

/* ========================================== *\
 * 		MESSAGE CALLBACKS
\* ========================================== */

void CameraConnector::depth_image_callback(const sensor_msgs::ImageConstPtr& msg)
{
	/*
	* converts the incoming ROS message to an OpenCV image
	*/

	try{
		// convert to cv: 32bit float representing meters (rostopic interprets as int8)
		cv_ptr_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);

		// image received - update status
		if(!running){
			std::cout << GREEN << "--- Receiving image stream" << RESET << endl;
			running = true;
		}
		if(display_depth_image){
			cv::imshow(OPENCV_WINDOW_DEPTH, cv_ptr_depth->image);
			cv::waitKey(3);
		}

	}catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

void CameraConnector::color_image_callback(const sensor_msgs::ImageConstPtr& msg)
{
	/*
	* converts the incoming ROS message to an OpenCV image
	*/

	try{
		cv_ptr_color = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

		// image received - update status
		if(!running){
			std::cout << GREEN << "--- Receiving image stream" << RESET << endl;
			running = true;
		}
		if(display_color_image){
			cv::imshow(OPENCV_WINDOW_COLOR, cv_ptr_color->image);
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
		std::cout << GREEN << "--- Receiving point cloud stream" << RESET << endl;
		pc_running = true;
	}

	// convert to XYZ PC
	pcl::PointCloud<pcl::PointXYZ>::Ptr mycloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*msg, *mycloud);

	if (display_cloud && ! pc_viewer_->wasStopped())
	{
		pc_viewer_->showCloud(mycloud);
	}

	if (SAVE_CLOUD)
	{
		std::string path = ros::package::getPath("dyn_3d_mod");

		string filename;
		cout << "Please enter a file name: ";
		cin >> filename;

		if(filename.find_last_of(".")!=-1){
			if(filename.substr(filename.find_last_of(".") + 1) != "pcd") {
				std::cout << "Invalid file name. Please try again." << std::endl;
				return;
			}
		}else{
			filename = filename + ".pcd";
		}

		pcl::io::savePCDFileASCII(path+"/recordings/"+filename, *mycloud);
		std::cout << GREEN << "--- Point cloud was saved to: " << path+"/recordings/"+filename << RESET << std::endl;
		SAVE_CLOUD = false;
	}

}

/* ========================================== *\
 * 		VISUALIZATION
\* ========================================== */

void CameraConnector::show_depth(bool show)
{
	if(show == true && display_depth_image == false){
		display_depth_image = true;
	}else if(show == false){
		display_depth_image = false;
		cv::destroyWindow(OPENCV_WINDOW_DEPTH);
	}
}
void CameraConnector::show_color(bool show)
{
	if(show == true && display_color_image == false){
		display_color_image = true;
	}else if(show == false){
		display_color_image = false;
		cv::destroyWindow(OPENCV_WINDOW_COLOR);
	}
}
void CameraConnector::show_pc(bool show)
{
	if(show == true && display_cloud == false){
		display_cloud = true;
		pc_viewer_ = createViewer();
	}else if(show == false){
		display_cloud = false;
		// TODO: destroy viewer
	}
}

void keyboardEventOccurred(const visualization::KeyboardEvent& event,void* nothing)
{
	if (event.getKeySym() == "Return" && event.keyDown()){
		SAVE_CLOUD = true;
	}
}

boost::shared_ptr<visualization::CloudViewer> createViewer(){
	boost::shared_ptr<visualization::CloudViewer> v
	(new visualization::CloudViewer("Point Cloud Viewer"));
	v->registerKeyboardCallback(keyboardEventOccurred);

	return (v);
}


/* ========================================== *\
 * 		FILTERING
\* ========================================== */

void CameraConnector::pc_voxel_callback( const pcl::PCLPointCloud2ConstPtr& msg  )
{

  pcl::PCLPointCloud2 cloud_filtered;
  pcl::VoxelGrid<pcl::PCLPointCloud2> vgrid;
  vgrid.setInputCloud (msg);
  vgrid.setLeafSize (0.1, 0.1, 0.1);
  vgrid.filter (cloud_filtered);

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

/* ========================================== *\
 * 		HELP
\* ========================================== */

void CameraConnector::display_help( )
{
	std::cout<<"\n";
	std::cout << "=================================" << std::endl;
	std::cout << " USAGE:" << std::endl;
	std::cout << "---------------------------------" << std::endl;
	std::cout << " [RETURN]: save point cloud" << std::endl;
	std::cout << "=================================" << std::endl;
	std::cout<<"\n";
}


