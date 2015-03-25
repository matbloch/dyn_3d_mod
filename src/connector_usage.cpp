#include <iostream>
#include "../lib/camera_connector.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <unistd.h>
#include <time.h>	// timing

#include "boost/program_options.hpp"

using namespace std;


int main(int argc, char* argv[])
{

		// init ros node
		ros::init(argc, argv, "dyn_3d_modeling");

		// create camera object
		CameraConnector *cam = new CameraConnector();

		ros::Rate r(10); // 10 Hz

		unsigned int timeout = 6;  // connection timeout in seconds
		time_t init_time = time(0);


		while (ros::ok())
		{

			// when camera is ready, do ...
			if(cam->running == true){
				cout << "The depth value is : " << cam->cv_ptr->image.at<float>(49,39) << endl;
			}else if(time(0) > timeout + init_time){
				// Connection timed out - terminate the node
				cout << "---Connection timed out" << endl;
				break;
			}

			ros::spinOnce();
			r.sleep();
		}

	return 0;


}
