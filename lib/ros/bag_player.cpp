#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/package.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"

#include <iostream>

#include "definitions.h"	// constants

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


int main(int argc, char **argv) {

	/*

	  Loads and publishes the following topics from a bag file:
	  /camera/depth/image (depth image)
	  /camera/depth/points (point cloud)

	*/

    ros::init(argc, argv, "rosbag_publisher");
    ros::start();
    ros::Rate loop_rate(30);

    // handles
    ros::NodeHandle nh;
    image_transport::ImageTransport it_(nh);

    // topics to load from the rosbag
    std::string bag_depth_img_msg_1 = "/camera1/depth/image";
    std::string bag_depth_img_msg_2 = "/camera2/depth/image";
    std::string bag_depth_points_msg_1 = "/camera1/depth/points";
    std::string bag_depth_points_msg_2 = "/camera2/depth/points";


    std::string bag_pc_msg_1 = "/camera1/depth/image";
    std::string bag_pc_msg_2 = "/camera2/depth/image";
    std::string bag_pc_msg_3 = "/camera1/depth/points";
    std::string bag_pc_msg_4 = "/camera2/depth/points";

    std::vector<std::string> topics;
    topics.push_back(bag_depth_img_msg_1);
    topics.push_back(bag_depth_img_msg_2);
    topics.push_back(bag_depth_points_msg_1);
    topics.push_back(bag_depth_points_msg_2);

    // prepare image publisher
    image_transport::Publisher depth_image_pub1_ = it_.advertise(bag_depth_img_msg_1, 1);
    image_transport::Publisher depth_image_pub2_ = it_.advertise(bag_depth_img_msg_2, 1);

    // prepare pc publisher: pc as original sensor message point cloud, not a pcl
    ros::Publisher pc_pub1_ = nh.advertise<sensor_msgs::PointCloud2> (bag_depth_points_msg_1, 1);
    ros::Publisher pc_pub2_ = nh.advertise<sensor_msgs::PointCloud2> (bag_depth_points_msg_2, 1);


    // specify file path
	std::string bag_path = ros::package::getPath("dyn_3d_mod");


	if(argc < 2){
		bag_path.append("/recordings/all_messages.bag");	// load standard file
	}else{
		bag_path.append(argv[1]);
	}

	std::cout << GREEN << "--- Bag player started with file:" << bag_path << RESET << std::endl;


  while (ros::ok()){

	rosbag::Bag bag;
	bag.open(bag_path, rosbag::bagmode::Read);
	rosbag::View view(bag, rosbag::TopicQuery(topics));

	// loop through messages
	foreach(rosbag::MessageInstance const m, view)
	{
		// publish depth image
		if (m.getTopic() == bag_depth_img_msg_1)
		{
			sensor_msgs::Image::ConstPtr depth_img_ptr = m.instantiate<sensor_msgs::Image>();
			if (depth_img_ptr != NULL){
				depth_image_pub1_.publish(*depth_img_ptr);
			}
		}
		if (m.getTopic() == bag_depth_img_msg_2)
		{
			sensor_msgs::Image::ConstPtr depth_img_ptr = m.instantiate<sensor_msgs::Image>();
			if (depth_img_ptr != NULL){
				depth_image_pub2_.publish(*depth_img_ptr);
			}
		}
		// publish point cloud
		if (m.getTopic() == bag_depth_points_msg_1)
		{
			sensor_msgs::PointCloud2::ConstPtr pc_ptr = m.instantiate<sensor_msgs::PointCloud2>();
			if (pc_ptr != NULL){
				pc_pub1_.publish(*pc_ptr);
			}
		}
		// publish point cloud
		if (m.getTopic() == bag_depth_points_msg_2)
		{
			sensor_msgs::PointCloud2::ConstPtr pc_ptr = m.instantiate<sensor_msgs::PointCloud2>();
			if (pc_ptr != NULL){
				pc_pub2_.publish(*pc_ptr);
			}
		}

		break;

	}

	bag.close();
	ros::spinOnce();
	loop_rate.sleep();

  }

}
