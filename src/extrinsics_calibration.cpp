#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


// OpenCV includes
#include <opencv2/nonfree/features2d.hpp>


// globals
bool display_features = true;
bool display_matched_features = true;



void calibration_callback(
		const sensor_msgs::ImageConstPtr& d1, const sensor_msgs::ImageConstPtr& d2,
		const sensor_msgs::ImageConstPtr& c1,const sensor_msgs::ImageConstPtr& c2,
		const sensor_msgs::PointCloud2ConstPtr& cloud1, const sensor_msgs::PointCloud2ConstPtr& cloud2)
{

	/*
	 * TODO:
	 * - Subscribe to both depth, color and point cloud streams
	 * 0. Do conversions
	 * 1. Wait for key to take snapshot of registered depth streams
	 * 2. Convert color images to greyscale
	 * 3. Detect (SIFT/FAST) in greyscale images
	 * 3.1 Display detected features l/r
	 * 4. Find corresponding features
	 * 4.1 Display corresponding features l/r lines
	 * 5. Get depth values corresponding the the matching features (read out depth values at feature position)
	 * 6. Calculate Transformation (Rotation & Translation) using RANSAC
	 * 7. Transform point cloud with RANSAC approximation
	 * 8. Use IPC to refine first guess and calculate second transformation
	 */


	ROS_INFO("Configuration initiated.");

	// allocate memory
	cv_bridge::CvImagePtr d1_ptr, d2_ptr, c1_ptr, c2_ptr;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    /* ========================================== *\
     * 		0. OpenCV & PCL conversion
    \* ========================================== */

	// Convert images to CV images
    try
    {
      d1_ptr = cv_bridge::toCvCopy(d1, sensor_msgs::image_encodings::TYPE_32FC1);
      d2_ptr = cv_bridge::toCvCopy(d2, sensor_msgs::image_encodings::TYPE_32FC1);
      d1_ptr = cv_bridge::toCvCopy(d1, sensor_msgs::image_encodings::BGR8);
      d2_ptr = cv_bridge::toCvCopy(d2, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Convert point clouds to PCL format
	pcl::fromROSMsg(*cloud1, *cloud1_ptr);
	pcl::fromROSMsg(*cloud2, *cloud2_ptr);

    /* ========================================== *\
     * 		0.1 Waiting to take snapshot
    \* ========================================== */

		// display images

		// wait for key

    /* ========================================== *\
     * 		1.Convert to greyscale
    \* ========================================== */



    /* ========================================== *\
     * 		2. Feature detection
    \* ========================================== */

	cv::SiftFeatureDetector detector;
	std::vector<cv::KeyPoint> keypoints1, keypoints2;

	// detect features
	detector.detect(gs1,keypoints1);
	detector.detect(gs2,keypoints2);

	// display results
	if(display_features){
		cv::Mat output1, output2;
		cv::drawKeypoints(gs1, keypoints1, output1);
		cv::drawKeypoints(gs2, keypoints2, output2);
		cv::imshow('Features', output1);
		cv::waitKey(0);
		cv::imshow('Features', output2);
		cv::waitKey(0);
	}

    /* ========================================== *\
     * 		3. Feature matching
    \* ========================================== */


    /* ========================================== *\
     * 		4. Publishing
    \* ========================================== */

	// convert back to ROS message
    pcl::toROSMsg(output_pcl, output);

    // publish the concatenated point cloud
    pub_.publish(output);

}


int main(int argc, char** argv)
{
	/*
	 * INPUT: camera streams (sensor image msg)
	 * OUTPUT: synced, aligned voxel structure (CV::Mat)
	 *
	 */

	ros::init(argc, argv, "dyn_3d_photo");
	ros::NodeHandle nh;


	// define subscribers
	// TODO: filter messages for image subscribers
	message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_1(nh, "/camera1/depth/points", 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_2(nh, "/camera2/depth/points", 1);

	// callback if message with same timestaps are received (buffer length: 10)
	sensor_msgs::ImageConstPtr

	// The policy merges kinect messages with approximately equal timestamp into one callback
	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::Image,sensor_msgs::Image,
			sensor_msgs::Image,sensor_msgs::Image,
			sensor_msgs::PointCloud2,sensor_msgs::PointCloud2>
	SyncPolicy;

	message_filters::Synchronizer< SyncPolicy > sync(SyncPolicy(10),
				depth_sub_1, depth_sub_2,
				color_sub_1, color_sub_2,
				pc_sub_1, pc_sub_2);

	// add callback
	sync.registerCallback(boost::bind(&calibration_callback, _1, _2));


	ROS_INFO("Configuration node initialized");


	while(ros::ok()){
	  	ros::spinOnce();
	}

	return 0;
}
