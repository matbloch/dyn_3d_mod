#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// pcl includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>

using namespace sensor_msgs;
using namespace message_filters;


// globals
ros::Publisher pc_pub_;
pcl::PointCloud<pcl::PointXYZ> pc_merged;


void processing_callback(const sensor_msgs::PointCloud2ConstPtr& cloud1, const sensor_msgs::PointCloud2ConstPtr& cloud2)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud2(new pcl::PointCloud<pcl::PointXYZ>);

	// convert to XYZ PCL pc
	pcl::fromROSMsg(*cloud1, *pcl_cloud1);
	pcl::fromROSMsg(*cloud2, *pcl_cloud2);

	/*
	 * TODO: frame transformation
	 *
	 *
	 */

	pc_merged = pcl_cloud1;
	pc_merged += pcl_cloud2;

	// convert back to ROS message
    pcl::toROSMsg(output_pcl, output);

    // publish the concatenated point cloud
    pc_pub_.publish(output);

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "pc_merger");
	ros::NodeHandle nh;

	// define subscribers
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_1(nh, "/camera1/depth/image", 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_2(nh, "/camera2/depth/image", 1);

	message_filters::Subscriber<Image> image_sub(nh, "image", 1);
	message_filters::Subscriber<CameraInfo> info_sub(nh, "camera_info", 1);

	// callback if message with same timestaps are received (buffer length: 10)
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> sync_policy;
	message_filters::Synchronizer<sync_policy> sync(MySyncPolicy(10), cloud_sub_1, cloud_sub_2);


	// sync
	TimeSynchronizer<Image, CameraInfo> sync(image_sub, info_sub, 10);

	// add callback
	sync.registerCallback(boost::bind(&processing_callback, _1, _2));

	// register publisher for the merged clouds
	pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>(ros::this_node::getName() + "/merged_cloud", 1);

	ROS_INFO("PC merger initalized.");


	while(ros::ok()){
	  	ros::spinOnce();
	}

	return 0;
}
