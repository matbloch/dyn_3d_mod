#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <pcl/io/pcd_io.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);


class CameraConnector
{
	private:

		ros::NodeHandle nh_;

		// point cloud
		ros::Subscriber pc_sub_;

	public:

		// point cloud pointer
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;


	CameraConnector(bool flag);
	~CameraConnector();

	int i=0;

	/* message callbacks */
	void pc_callback ( const sensor_msgs::PointCloud2ConstPtr& msg );

};

/* ========================================== *\
 * 		CONSTRUCTOR/DESTRUCTOR
\* ========================================== */

/* constructor */
CameraConnector::CameraConnector(bool flag):
	nh_("~")
{

  // Subscribe to depth point cloud
  pc_sub_ = nh_.subscribe ("/camera/depth/points", 1, &CameraConnector::pc_callback, this);

}

CameraConnector::~CameraConnector(){}

/* ========================================== *\
 * 		MESSAGE CALLBACKS
\* ========================================== */

void CameraConnector::pc_callback ( const sensor_msgs::PointCloud2ConstPtr& msg )
{

	// convert to XYZ PC
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*msg, *cloud);


	//cloud_ptr = cloud;

}

/* ========================================== *\
 * 		MAIN FUNCTION
\* ========================================== */


int main(int argc, char** argv)
{


	pcl::visualization::CloudViewer viewer_("Cloud Viewer");

    /* ROS stuff */
    ros::init(argc, argv, "dyn_3d_modeling");
    ros::start();
    ros::Rate r(10);


	// create camera object
	CameraConnector cam = new CameraConnector(true);

	int i = 0;
	while (ros::ok())
	{


		/* visualize point cloud */
		viewer_.showCloud(cloud);

		/*
		if(i==10){

		}
		*/

		ros::spinOnce();
		r.sleep();

	}	// stop when camera node handle is shut down


	return 0;

}
