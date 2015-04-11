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
#include "../lib/camera_connector.h"	// Kinect camera connector
#include "../lib/definitions.h"
#include "../lib/filters.h"	  // image filters

#include "pcl_connector.cpp"	  // image filters

namespace
{
  const size_t ERROR_IN_COMMAND_LINE = 1;
  const size_t ERROR_UNHANDLED_EXCEPTION = 2;

}

int main(int argc, char** argv)
{

  try
  {
    namespace po = boost::program_options;
    po::options_description desc("Options");

    /* ========================================== *\
     * 		SETUP OPTIONS
    \* ========================================== */

    desc.add_options()
      ("help,h", "Print help messages") // can use -h
      ("cv,c", "Perform OpenCV tests")	// display the depth stream
      ("pcl,p", "Display point cloud");

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
    ros::init(argc, argv, "dyn_3d_modeling");	// start node
    ros::start();
    ros::Rate r(30); // 30 Hz - Kinect: 30fps


    if(vm.count("cv")){

		// create camera object
		CameraConnector *cam = new CameraConnector(true);

		unsigned int timeout = 6;  // connection timeout in seconds
		time_t init_time = time(0);

		while (ros::ok())
		{

			// when camera is ready, display the stream
			if(cam->running == true){

				/*
				 * do something with the CV image (use: cam->cv_ptr->image)
				 *
				 */

			}else if(time(0) > timeout + init_time){

				std::cout << RED << "--- Connection timed out" << RESET << std::endl;
				break; // Connection timed out - terminate the node

			}

			ros::spinOnce();
			r.sleep();

		}	// stop when camera node handle is shut down


    }else if(vm.count("pcl")){

		CameraConnector cam(true);

		while (ros::ok())
		{

			/*
			 * perform pcl tests
			 *
			 */

			ros::spinOnce();
			r.sleep();

		}

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

  return 0;


}
