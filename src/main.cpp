#include "boost/program_options.hpp"	// linking in CMakeLists.txt



#include <iostream>
#include <string>

/* OpenCV 2 */
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <unistd.h>
#include <time.h>	// timing

/* Include own libraries */
#include "../lib/camera_connector.h"	// Kinect camera connector
#include "../lib/definitions.h"

namespace
{
  const size_t ERROR_IN_COMMAND_LINE = 1;
  const size_t SUCCESS = 0;
  const size_t ERROR_UNHANDLED_EXCEPTION = 2;

}

int main(int argc, char** argv)
{

/* ROS stuff */
ros::init(argc, argv, "dyn_3d_modeling");	// start node
ros::start();
ros::Rate r(30); // 30 Hz - Kinect: 30fps


  try
  {
    namespace po = boost::program_options;
    po::options_description desc("Options");

    /* ========================================== *\
     * 		SETUP OPTIONS
    \* ========================================== */

    desc.add_options()
      ("help,h", "Print help messages") // can use -h
      ("display,d", "Displays the depth stream using OpenCV")	// display the depth stream
      ("filter,f", "Add filters to the depthstream");			// test filters

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
        		  << "Dynamic 3D modeling using a spatiotemporal Voxel Octrees representation"
        		  << std::endl
                  << desc << std::endl;	// display the options descriptions (desc)

        return SUCCESS;
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

    static const std::string OPENCV_WINDOW = "Image window";


    if(vm.count("display")){

		// create camera object
		CameraConnector *cam = new CameraConnector();

		unsigned int timeout = 6;  // connection timeout in seconds
		time_t init_time = time(0);

		cout << "--- Trying to connect to the camera..." << endl;

		while (ros::ok())
		{

			// when camera is ready, display the stream
			if(cam->running == true){

				cv::imshow(OPENCV_WINDOW, cam->cv_ptr->image);
				cv::waitKey(3);

			}else if(time(0) > timeout + init_time){

				cout << "--- Connection timed out" << endl;
				break; // Connection timed out - terminate the node

			}

			ros::spinOnce();
			r.sleep();

		}	// stop when camera node handle is shut down

		cv::destroyWindow(OPENCV_WINDOW);	// destroy opencv window

    }else if(vm.count("filter")){

		// create camera object
		CameraConnector *cam = new CameraConnector();
		cv::Mat dst;	// blurred image

		unsigned int timeout = 6;  // connection timeout in seconds
		time_t init_time = time(0);

		cout << "--- Trying to connect to the camera..." << endl;

		while (ros::ok())
		{

			// when camera is ready, display the stream
			if(cam->running == true){

				// apply gaussian blur with kernel size 11
				GaussianBlur( cam->cv_ptr->image, dst, cv::Size( 11, 11 ), 0, 0 );
				cv::imshow(OPENCV_WINDOW, dst);
				cv::waitKey(3);

			}else if(time(0) > timeout + init_time){

				cout << "--- Connection timed out" << endl;
				break; // Connection timed out - terminate the node

			}

			ros::spinOnce();
			r.sleep();

		}	// stop when camera node handle is shut down

		cv::destroyWindow(OPENCV_WINDOW);	// destroy opencv window


    }


    // show down node
    ros::shutdown();


  }
  catch(std::exception& e)
  {
    std::cerr << "Unhandled Exception reached the top of main: "
              << e.what() << ", application will now exit" << std::endl;
    return ERROR_UNHANDLED_EXCEPTION;

  }

  return SUCCESS;


}
