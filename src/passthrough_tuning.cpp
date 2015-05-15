#include <ros/ros.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

// Opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int x_from = 900;
int x_to = 900;
int y_from = 900;
int y_to = 900;
int z_from = 900;
int z_to = 900;

pcl::visualization::PCLVisualizer viewer ("Segmentation preview");

void callback(const sensor_msgs::PointCloud2ConstPtr& msg){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // Convert point clouds to PCL format
	pcl::fromROSMsg(*msg, *cloud);

	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-0.01*((float)y_from), 0.01*((float)y_to));
	pass.filter (*cloud);

	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (-0.01*((float)x_from), 0.01*((float)x_to));
	pass.filter (*cloud);

	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0, 0.01*((float)z_to));
	pass.filter (*cloud);

	// Visualizing pointcloud
	viewer.addPointCloud (cloud, "scene_cloud");
	viewer.spinOnce();
	viewer.removePointCloud("scene_cloud");
	cv::waitKey(3);

}

int main (int argc, char** argv){

	ros::init(argc, argv, "sub_pcl");
	ros::NodeHandle nh_;


	std::string camera_name;

	// resolve topics
	if(argc > 1){
		camera_name = nh_.resolveName(argv[1]);
	}else{
		std::cout << "Please provide the camera name as an argument. Example: camera1" << std::endl;
		return -1;
	}

	ros::Subscriber pc_sub_ = nh_.subscribe("/"+camera_name+"/depth/points", 1, callback);

	cv::namedWindow( "PassThrough filter parameters");
	cvCreateTrackbar("X from", "PassThrough filter parameters", &x_from, 900, NULL);
	cvCreateTrackbar("X to", "PassThrough filter parameters", &x_to, 900, NULL);
	cvCreateTrackbar("Y from [cm]", "PassThrough filter parameters", &y_from, 900, NULL);
	cvCreateTrackbar("Y to [cm]", "PassThrough filter parameters", &y_to, 900, NULL);
	cvCreateTrackbar("Z max", "PassThrough filter parameters", &z_to, 900, NULL);
	cv::waitKey(3);

    /* ROS stuff */
    ros::init(argc, argv, "PC PassThrough filter tuning");	// start node
    ros::start();
    ros::Rate r(30); // 30 Hz - Kinect: 30fps

	while (ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}

	return (0);
}
