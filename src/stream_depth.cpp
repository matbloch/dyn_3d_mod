#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include <thread>
#include <chrono>
#include <time.h>
 
using namespace std;

static const std::string OPENCV_WINDOW = "Image window"; // window name

class CameraConnector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;    // publish the cv image as ros message service
  
  // stream is connected
  bool running = false;
  unsigned int refresh_rate = 30;   // stream pick up rate
  unsigned int timeout = 6;
  
  // depth map
  cv_bridge::CvImagePtr cv_ptr;   // access the image by cv_ptr->image
  
public:
  CameraConnector()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/depth/image", 1, 
      &CameraConnector::depth_callback, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    // create OpenCV window
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~CameraConnector()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void depth_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    
    try
    {
      // convert to cv: 32bit float representing meters (rostopic interpretes as int8)
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    // update status
    

    // display the depth value
    cout << "The depth value is : " << cv_ptr->image.at<float>(49,39) << endl;
    
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
  }
  
  void wait_for_connection()
  {
      
      time_t now = time(0);
      
      // set timeout
      while(true){
          
          if(running){
              cout << "---Receiving camera stream" << endl;
              break;
          }elseif(time(0) > timeout){
              cout << "---Connection timed out" << endl;
              
          }
            // wait
            this_thread::sleep_for(chrono::seconds(timeout));
      }
      
  }
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  CameraConnector myCam;
  ros::spin();
  return 0;
}
