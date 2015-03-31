#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/package.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"

#include "definitions.h"	// constants

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


int main(int argc, char **argv) {

    ros::init(argc, argv, "publish_scene");
    ros::start();
    ros::Rate loop_rate(30);

    ros::NodeHandle nh;
    image_transport::ImageTransport it_(nh);
    // topic to publish (use the original name)
    image_transport::Publisher image_pub_ = it_.advertise("/camera/depth/image", 1);

    // topic to load from the rosbag
    std::string bag_camera_msg = "/camera/depth/image";
    std::vector<std::string> topics;
    topics.push_back(bag_camera_msg);

    // specify file path
	std::string bag_path = ros::package::getPath("dyn_3d_mod");
	bag_path.append("/recordings/all_messages.bag");

	std::cout << GREEN << "--- Bag player started" << RESET << std::endl;


  while (ros::ok()){

	rosbag::Bag bag;
	bag.open(bag_path, rosbag::bagmode::Read);
	rosbag::View view(bag, rosbag::TopicQuery(topics));

	// loop through messages
	foreach(rosbag::MessageInstance const m, view)
	{
		if (m.getTopic() == bag_camera_msg)
		{
			sensor_msgs::Image::ConstPtr rgb_image_ptr = m.instantiate<sensor_msgs::Image>();
			if (rgb_image_ptr != NULL){
				image_pub_.publish(*rgb_image_ptr);
			}
			break;
		}

	}

	bag.close();
	ros::spinOnce();
	loop_rate.sleep();

  }

}
