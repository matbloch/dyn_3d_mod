#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/package.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <sstream>



int main(int argc, char **argv) {

    ros::init(argc, argv, "publish_scene");
    ros::start();
    ros::Rate loop_rate(10);



    ros::NodeHandle nh;
    image_transport::ImageTransport it_(nh);
    image_transport::Publisher image_pub_ = it_.advertise("/camera/depth/image", 1);

    // topic in the rosbag
    std::string bag_camera_msg = "/camera/depth/image";
    std::vector<std::string> topics;
    topics.push_back(bag_camera_msg);

    // file path
	std::string path = ros::package::getPath("dyn_3d_mod");
	path.append("/recordings/stream.bag");

	std::cout << path << std::endl;


  while (ros::ok()){

    rosbag::Bag bag;
    bag.open(path, rosbag::bagmode::Read);


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
      }

    }

  bag.close();
  ros::spinOnce();
  loop_rate.sleep();

  }

}
