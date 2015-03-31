#include <ros/ros.h>
// ROS core
#include <ros/package.h>
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>


#include <iostream>


#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH





// pcl - ros
/*
#include <pcl_ros>
#include <pcl_ros/point_cloud.h>
*/

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


#include <pcl_ros/point_cloud.h>


#include <pcl_ros/transforms.h>


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


		uint32_t queue_size;

	CameraConnector(): it_(nh_)
	{

		queue_size = 1;

		// topic to subscribe to
		std::string topic = nh_.resolveName("/camera/depth/points");


		// to create a subscriber, you can do this (as above):
		ros::Subscriber sub = nh_.subscribe<sensor_msgs::PointCloud2> (topic, queue_size, &CameraConnector::pc_callback, this);


	}
	~CameraConnector()
	{

	}

	void pc_callback(const sensor_msgs::PointCloud2ConstPtr& pc2)
	{

		std::cout << "test" << std::endl;

		pcl::PCLPointCloud2::Ptr pcl_pc(new pcl::PCLPointCloud2);

		// Transformation into PCL type PointCloud2
		pcl_conversions::toPCL(*(pc2), *(pcl_pc));



		std::cout << pcl_pc << std::endl;
		/*
		 *
		 * pcl::toROSMsg(pcl_pc, pc2);
		 *
		 *
		 *
		 */

		/*
		  pcl::PCLPointCloud2 pcl_pc;
		  pcl_conversions::toPCL(input, pcl_pc);

		  pcl::PointCloud<pcl::PointXYZ> cloud;

		  pcl::fromPCLPointCloud2(pcl_pc, cloud);
		  pcl::YOUR_PCL_FUNCTION(cloud,...);


		 */
	}

	void pcl_conversion_cv (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
	{
		/*
	  // Container for original & filtered data
	  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	  pcl::PCLPointCloud2 cloud_filtered;

	  // Convert to PCL data type
	  pcl_conversions::toPCL(*cloud_msg, *cloud);

	  // Perform the actual filtering
	  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	  sor.setInputCloud (cloudPtr);
	  sor.setLeafSize (0.1, 0.1, 0.1);
	  sor.filter (cloud_filtered);

	  // Convert to ROS data type
	  sensor_msgs::PointCloud2 output;
	  pcl_conversions::fromPCL(cloud_filtered, output);

	  // Publish the data
	  pub.publish (output);
	  */


		/*
		  // Convert to ROS data type
		  sensor_msgs::PointCloud2 output;
		  pcl_conversions::fromPCL(cloud_filtered, output);

		 */







	}

};



int main(int argc, char** argv)
{


	/* ROS stuff */
	ros::init(argc, argv, "sub_pcl");	// start node
	ros::start();
	ros::Rate r(30); // 30 Hz - Kinect: 30fps


	// create camera object
	CameraConnector *cam = new CameraConnector();


	while (ros::ok())
	{


		ros::spinOnce();
		r.sleep();

	}



}

