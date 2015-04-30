// OpenCV includes
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <cv_bridge/cv_bridge.h>

// Configure library
#include <config/config_handler.h>

ConfigHandler conf;
using namespace cv;

/*



Writing to file

cv::FileStorage storage("test.yml", cv::FileStorage::WRITE);
storage << "img" << img;
storage.release();

Reading from file

cv::FileStorage storage("test.yml", cv::FileStorage::READ);
storage["img"] >> img;
storage.release();


 */

void loadMat(cv::Mat &m, const char* filename = "matrix.yml")
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened()){
	 fs.open(filename, cv::FileStorage::READ);
	 fs["matrix"] >> m;
	 fs.release();
	}
}

int main(int argc, char** argv)
{
cv::Mat mat1;
cv::Mat mat2;

cv::FileStorage storage("test/img1.yml", cv::FileStorage::READ);
storage["img"] >> mat1;
storage.release();
cv::FileStorage storage2("test/img2.yml", cv::FileStorage::READ);
storage2["img"] >> mat2;
storage2.release();


cv::namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
imshow( "Display window", mat1 );                   // Show our image inside it.

cv::waitKey(0);

}
