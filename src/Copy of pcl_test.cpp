#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "definitions.h"	// constants


// ROS core
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <pcl_ros/point_cloud.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

// Need to include the pcl ros utilities
#include "pcl_ros/point_cloud.h"

// PCL includes
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl_visualization/pcl_visualizer.h>


using namespace std;

class PointCloudConnector
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

		// camera status
		bool running;

	PointCloudConnector(): it_(nh_)
	{
	  running = false;
	  image_sub_ = it_.subscribe("/camera/depth/points", 1, &PointCloudConnector::pc_callback, this);
	}
	~PointCloudConnector()
	{

	}

	void pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
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



	// Main internal function to process the point cloud message
	void ProcessCloudMessage()
	{

		usleep (10000);
		viewer->spinOnce (10);
		// if no new messages appear, just stop processing
		if (!cloud_msg || cloud_msg == old_cloud_msg)
			return;
		old_cloud_msg = cloud_msg;

		// get message from ROS and convert to point cloud
		PointCloud<PointXYZRGB> point_cloud;
		fromROSMsg(*cloud_msg, point_cloud);

		// if the sensor point cloud provides "far range data", use it to compute the sensor_pose
		PointCloud<PointWithViewpoint> far_ranges;
		RangeImage::extractFarRanges(*cloud_msg, far_ranges);
		if (pcl::getFieldIndex(*cloud_msg, "vp_x")>=0)
		{
			PointCloud<PointWithViewpoint> tmp_pc;
			fromROSMsg(*cloud_msg, tmp_pc);
			Eigen::Vector3f average_viewpoint = RangeImage::getAverageViewPoint(tmp_pc);
			sensor_pose = Eigen::Translation3f(average_viewpoint) * sensor_pose;
		}

		//ROS_INFO("(h,w)=%d,%d.\n", point_cloud.height, point_cloud.width);

		// For efficieny, all functions in PCL work with PointCloud<PointT>::Ptr, so we extract them from the real point cloud
		PointCloud<PointXYZRGB>::Ptr point_cloud_ptr (new PointCloud<PointXYZRGB>(point_cloud));
		PointCloud<PointXYZRGB>::Ptr point_cloudfilt_ptr (new PointCloud<PointXYZRGB>); // filtered pc

		// Filter clouds in Z dimension (min, max)
		FilterPointCloudZ(point_cloud_ptr, point_cloudfilt_ptr, 0.0f, 10.0f);

		// visualize
		VisualizePointCloud(point_cloudfilt_ptr);


	}


	// Visualize Point Clouds
	void VisualizePointCloud(PointCloud<PointXYZRGB>::Ptr& pc)
	{
		viewer->removePointCloud ("range image cloud");

		if(viz_type == 0)
		{

			range_image->createFromPointCloud(*pc, angular_resolution, deg2rad(360.0f), deg2rad(180.0f),
		                                sensor_pose, coordinate_frame, noise_level, min_range, border_size);

			// display DEPTH point cloud
		    	pcl_visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> color_handler_cloud(*range_image,200, 200, 200);
			viewer->addPointCloud (*range_image, color_handler_cloud, "range image cloud");

		}
		else
		{

			// display RGB point cloud
			pcl_visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_handler_cloud(*pc);
		    	viewer->addPointCloud (*pc, color_handler_cloud, "range image cloud");

			// get a sample data point from the point cloud
			PointXYZRGB tpoint;
			tpoint = pc->at(200,200); // sample image point
			std::vector<uint8_t> rgb_t(4); // hold 4 elements
			extract_pcl_rgb(tpoint, rgb_t);
			// x, y, z in meters. x right, y down, z forward from camera center, r, g, b:(1 - 255)
			ROS_INFO("(x,y,z)=%g,%g,%g, (r,g,b)=%d,%d,%d", tpoint.x, tpoint.y, tpoint.z, rgb_t[0],rgb_t[1],rgb_t[2]);


		}
	}

	// function to extract rgb stored in pcl PointXYZRGB
	void extract_pcl_rgb(pcl::PointXYZRGB& point_t, std::vector<uint8_t>& rgb_v)
	{
		// extract color values
		uint32_t rgb_val_;
		memcpy(&rgb_val_, &(point_t.rgb), sizeof(float));

		uint8_t garbage_ = (uint8_t)((rgb_val_ >> 24) & 0x000000ff);
		uint8_t r_ = (uint8_t)((rgb_val_ >> 16) & 0x000000ff);
		uint8_t g_ = (uint8_t)((rgb_val_ >> 8) & 0x000000ff);
		uint8_t b_ = (uint8_t)((rgb_val_) & 0x000000ff);

		// save to rgb vector
		rgb_v[0]=r_;
		rgb_v[1]=g_;
		rgb_v[2]=b_;
		rgb_v[3]= garbage_;

	}


};

int main(int argc, char** argv)
{

  ros::init (argc, argv, "pcl test");
  PointCloudConnector pcc();
  ros::spin();
  return 0;

}

