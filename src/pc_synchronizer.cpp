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
// PCL features
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>


// Message filters
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/sync_policies/exact_time.h>		// exact time synchronization
#include <message_filters/sync_policies/approximate_time.h>	// approximate time synchronization


using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;
using namespace std;



// globals
bool display_features = true;
bool display_matched_features = true;
string feature_detector = "SURF";

int avg_nr = 5;


// combine two identical size images into one big image
void combineTwoImages(cv::Mat &dst, const cv::Mat img1, const cv::Mat img2){
   if (img1.cols != img2.cols || img1.rows != img2.rows) {
       return;
   }
   int rows = img1.rows;
   int cols = img1.cols;
   dst = cvCreateMat(rows, 2 * cols, img1.type());
   cv::Mat tmp = dst(cv::Rect(0, 0, cols, rows));
   img1.copyTo(tmp);
   tmp = dst(cv::Rect(cols, 0, cols, rows));
   img2.copyTo(tmp);
}


void synced_pc_callback(
		const sensor_msgs::PointCloud2ConstPtr& cloud1, const sensor_msgs::PointCloud2ConstPtr& cloud2
)
{

	std::cout << "starting detection" << std::endl;

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


	// convert to XYZ PC
	pcl::PointCloud<pcl::PointXYZ>::Ptr mycloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr mycloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*cloud1, *mycloud1);
	pcl::fromROSMsg(*cloud2, *mycloud2);

	//std::cout << "Cluster has " << mycloud->points.size() << " points." << std::endl;	// 640x480 points (kinect depth resolution)




	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals1(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the SHOT descriptors for each point.
	pcl::PointCloud<pcl::SHOT352>::Ptr descriptors1(new pcl::PointCloud<pcl::SHOT352>());
	pcl::PointCloud<pcl::SHOT352>::Ptr descriptors2(new pcl::PointCloud<pcl::SHOT352>());




    /* ========================================== *\
     * 		FEATURE DETECTION FOR FIRST CLOUD
    \* ========================================== */

	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(mycloud1);
	normalEstimation.setRadiusSearch(0.5);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals1);


	std::cout << "normals have been computed" << std::endl;

	// SHOT estimation object.
	pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
	shot.setInputCloud(mycloud1);
	shot.setInputNormals(normals1);
	// The radius that defines which of the keypoint's neighbors are described.
	// If too large, there may be clutter, and if too small, not enough points may be found.
	shot.setRadiusSearch(0.5);

	shot.compute(*descriptors1);


    /* ========================================== *\
     * 		VISUALIZE
    \* ========================================== */


	std::cout << "finished detection" << std::endl;

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

	ros::Rate r(100);

    /* ========================================== *\
     * 		2 - WORKING
    \* ========================================== */


	// define subscribers

	message_filters::Subscriber<sensor_msgs::PointCloud2> depth_sub_1(nh_, "/camera1/depth/points", 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> depth_sub_2(nh_, "/camera2/depth/points", 1);

	/*
	message_filters::Subscriber<sensor_msgs::PointCloud2> depth_sub_1(nh_, "/camera1/depth_registered/points", 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> depth_sub_2(nh_, "/camera2/depth_registered/points", 1);
	*/

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
															sensor_msgs::PointCloud2>
	ApproximateSyncPolicy;


	message_filters::Synchronizer<ApproximateSyncPolicy> sync(ApproximateSyncPolicy(10),
			depth_sub_1
			,depth_sub_2
			);
	  sync.registerCallback(boost::bind(&synced_pc_callback,
			  _1
			  ,_2
			  ));



	ROS_INFO("PC Synchronizer");


	while (ros::ok())
	{

	  ros::spinOnce();                   // Handle ROS events
	  r.sleep();
	}

	return 0;
}
