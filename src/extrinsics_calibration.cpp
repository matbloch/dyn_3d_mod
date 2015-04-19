#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

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

#include "../lib/pcl/pc_aligner.h"


// globals
bool display_features = true;
bool display_matched_features = true;

using namespace sensor_msgs;
using namespace message_filters;

int i=1;

void calibration_callback(
		const sensor_msgs::PointCloud2ConstPtr& c1, const sensor_msgs::PointCloud2ConstPtr& c2)
{

	if(i != 1){
		return;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);

    // Convert point clouds to PCL format
	pcl::fromROSMsg(*c1, *cloud1);
	pcl::fromROSMsg(*c2, *cloud2);

	// do alignment
	PCAligner aligner;
	aligner.setInputClouds(cloud1, cloud2);
	aligner.setLeafSize(0.02);
	aligner.setNormalEstSearchRadius(0.2);
	aligner.setICPMaximumCorrelationDist(0.1);
	aligner.setSHOTSearchRadius(0.1);
	aligner.startAlignment();

	i++;
}



int main(int argc, char** argv)
{
	/*
	 * INPUT: camera streams (sensor image msg)
	 * OUTPUT: synced, aligned voxel structure (CV::Mat)
	 *
	 */

	ros::init(argc, argv, "dyn_3d_photo");
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_(nh_);

	// define subscribers
	message_filters::Subscriber<sensor_msgs::PointCloud2> depth_sub_1(nh_, "/camera1/depth/points", 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> depth_sub_2(nh_, "/camera2/depth/points", 1);

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


	  ros::spin();

	  return 0;
}
