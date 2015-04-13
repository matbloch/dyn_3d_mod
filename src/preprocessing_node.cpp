#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


// globals
ros::Publisher pub_;

void preprocessing_callback(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2)
{

	/*
	 * TODO:
	 * 1. frame transformation
	 * 2. Voxel grid
	 * 3. Fusion
	 * 4. Publish as custom message
	 */


	ROS_INFO("Synced callback was called.");

    /* ========================================== *\
     * 		0. OpenCV conversion
    \* ========================================== */

	cv_bridge::CvImagePtr cv_ptr1;
	cv_bridge::CvImagePtr cv_ptr2;

    try
    {
      cv_ptr1 = cv_bridge::toCvCopy(msg1, sensor_msgs::image_encodings::TYPE_32FC1);
      cv_ptr2 = cv_bridge::toCvCopy(msg2, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    /* ========================================== *\
     * 		1. Frame transformation
    \* ========================================== */

    /*
     * cv_ptr1->image ...
     *
     */


    /* ========================================== *\
     * 		2. Voxel grid
    \* ========================================== */



    /* ========================================== *\
     * 		3. Fusion
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
	message_filters::Subscriber<sensor_msgs::PointCloud2> depth_sub_1(nh, "/camera1/depth/image", 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> depth_sub_2(nh, "/camera2/depth/image", 1);

	// callback if message with same timestaps are received (buffer length: 10)
	sensor_msgs::ImageConstPtr

	//The policy merges kinect messages with approximately equal timestamp into one callback
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
	                                                        sensor_msgs::Image>
	SyncPolicy;

	message_filters::Synchronizer< SyncPolicy > sync(SyncPolicy(10), depth_sub_1, depth_sub_2);

	// add callback
	sync.registerCallback(boost::bind(&preprocessing_callback, _1, _2));

	// register publisher for the merged clouds
	pub_ = nh.advertise<sensor_msgs::PointCloud2>(ros::this_node::getName() + "/preprocessed_data", 1);

	ROS_INFO("Preprocessing node initalized.");


	while(ros::ok()){
	  	ros::spinOnce();
	}

	return 0;
}
