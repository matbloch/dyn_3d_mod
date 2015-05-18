#include "boost/program_options.hpp"	// linking in CMakeLists.txt
#include <ros/package.h>


#include <iostream>
#include <string>

/* OpenCV 2 */
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <unistd.h>
#include <time.h>	// timing

/* Include own libraries */
#include "camera_connector.h"	// Kinect camera connector
#include "definitions.h"
#include "cv/filters.h"	  // image filters


namespace
{
  const size_t ERROR_IN_COMMAND_LINE = 1;
  const size_t ERROR_UNHANDLED_EXCEPTION = 2;

}

class IRSnapshot
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub1_;
	image_transport::Subscriber image_sub2_;
	std::string image_name1 = "Image 1";
	std::string image_name2 = "Image 2";
	int nr_images1;
	int nr_images2;
	int count;

public:
	IRSnapshot(int nr_images, std::vector<std::string> image_topics): it_(nh_)
	{

		if ( image_topics.size() > 2 ) {
			std::cout << "Sorry, only two image topics are supported at the moment." << std::endl;
			return;
		}

		// Subscrive to input video feed and publish output video feed
		if ( image_topics.size() >=1 ) {
			image_sub1_ = it_.subscribe(image_topics[0], 1,&IRSnapshot::imageCb1, this);
			 cv::namedWindow( image_name1, cv::WINDOW_AUTOSIZE );// Create a window for display.
		}
		if ( image_topics.size() >= 2 ) {
			image_sub2_ = it_.subscribe(image_topics[1], 1,&IRSnapshot::imageCb2, this);
			 cv::namedWindow( image_name2, cv::WINDOW_AUTOSIZE );// Create a window for display.
		}

		count = nr_images;
		nr_images1 = nr_images2 = 0;


	}

		~IRSnapshot()
		{
			//cv::destroyWindow(OPENCV_WINDOW);
		}



		void imageCb1(const sensor_msgs::ImageConstPtr& msg)
		{

			if(nr_images1 >= count){
				return;
			}

			cv_bridge::CvImagePtr cv_ptr;
			try
			{
			  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			}
			catch (cv_bridge::Exception& e)
			{
			  ROS_ERROR("cv_bridge exception: %s", e.what());
			  return;
			}

			// Update GUI Window
			cv::imshow(image_name2, cv_ptr->image);
			cv::waitKey(3);

			// save image
		    std::ostringstream ss;
		    ss << nr_images1;
			cv::imwrite( "camera1_"+ss.str()+".jpg", cv_ptr->image );

			nr_images1++;
		}

		void imageCb2(const sensor_msgs::ImageConstPtr& msg)
		{

			if(nr_images2 >= count){
				return;
			}

			cv_bridge::CvImagePtr cv_ptr;
			try
			{
			  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			}
			catch (cv_bridge::Exception& e)
			{
			  ROS_ERROR("cv_bridge exception: %s", e.what());
			  return;
			}

			// Update GUI Window
			cv::imshow(image_name2, cv_ptr->image);
			cv::waitKey(3);

			// save image
		    std::ostringstream ss;
		    ss << nr_images2;
			cv::imwrite( "camera2_"+ss.str()+".jpg", cv_ptr->image );

			nr_images2++;

		}
};


int main(int argc, char** argv)
{

  try
  {
    namespace po = boost::program_options;
    po::options_description desc("Options");
	std::vector<std::string> image_topics;
	float rate;
	int count;

    /* ========================================== *\
     * 		SETUP OPTIONS
    \* ========================================== */

    desc.add_options()
      ("help,h", "Print help messages") // can use -h
	  ("rate", po::value<float>(&rate)->required(), "Rate at which images are taken")
	  ("count", po::value<int>(&count)->required(), "The number of images that should be taken")
	  ("topics", po::value<std::vector<string> >(&image_topics)->required(), "image topics to record");

    po::variables_map vm;

    /* ========================================== *\
     * 		PARSE OPTIONS
    \* ========================================== */

    try
    {
      po::store(po::parse_command_line(argc, argv, desc),vm);

      if (vm.count("help"))
      {
    	// display the program options
        std::cout << std::endl
        		  << "Recording tool for image series."
        		  << std::endl
                  << desc << std::endl	// display the options descriptions (desc)
        		  << "\nImage messages:" << std::endl
				  << "\n/camera/ir/image_raw   -  IR image" << std::endl
				  << "\n/camera/depth/image_raw   -   Depth image" << std::endl;

        return 0;
      }

      po::notify(vm);
    }
    catch(po::error& e)
    {
      std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
      std::cerr << desc << std::endl;
      return ERROR_IN_COMMAND_LINE;
    }

    /* ========================================== *\
     * 		APPLICATION CODE
    \* ========================================== */



    /* ROS stuff */
    ros::init(argc, argv, "snapshot");	// start node
    ros::start();
    ros::Rate r(rate); // 30 Hz - Kinect: 30fps

    IRSnapshot IRS(count, image_topics);

	while (ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}

  }
  catch(std::exception& e)
  {
    std::cerr << "Unhandled Exception reached the top of main: "
              << e.what() << ", application will now exit" << std::endl;
    return ERROR_UNHANDLED_EXCEPTION;

  }

  return 0;

}
