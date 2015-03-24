#include <iostream>
#include "../lib/camera_connector.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <unistd.h>
#include <time.h>	// timing


using namespace std;


int main(int argc, char *argv[])
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

    	// Connection timed out - terminate the node
    	if(time(0) > timeout + init_time){
    		cout << "---Connection timed out" << endl;
    		break;
    	}

		// when camera is ready, do ...
		if(cam->running == true){
			cout << "The depth value is : " << cam->cv_ptr->image.at<float>(49,39) << endl;
		}

		ros::spinOnce();
		r.sleep();
    }

	return 0;


}
