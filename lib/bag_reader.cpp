#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <vector>
#include <iostream>



class readDepthBag
{
	std::string depth_image_topic;

	public:
		void setLength( double len );
		double getLength( void );
		readDepthBag(char*, std::vector<cv::Mat>&);  // This is the constructor

};


// constructor
readDepthBag::readDepthBag(char* filename, std::vector<cv::Mat>& images)
{

	/* settings */
	depth_image_topic = "/camera/depth/image";

	std::cout << "Reading bag..." << std::endl;

	// load rosbag
	rosbag::Bag bag;
	bag.open(filename, rosbag::bagmode::Read);

	// make topics
	std::vector<std::string> topics;
	topics.push_back(depth_image_topic);

	// view topics in bag
	rosbag::View view(bag, rosbag::TopicQuery(topics));


	foreach(rosbag::MessageInstance const m, view)
	{

		if(m.getTopic() == depth_image_topic) {
			sensor_msgs::Image::ConstPtr msg = m.instantiate<sensor_msgs::Image>();

			// convert to cv: 32bit float representing meters (rostopic interprets as int8)
			cv_bridge::CvImagePtr cv_ptr;
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);

			// output
			images.push_back(cv_ptr->image);
		}

	}

	bag.close();
}
