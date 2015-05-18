/*
 * script.cpp
 *
 *  Created on: 16 mrt. 2015
 *      Author: RGrandia
 */

#include <iostream>
#include <ros/package.h>

/* OpenCV 2 */
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.hpp>
#include <math.h>

// Eigen
#include <Eigen/Dense>


int	GRID_SIZE = 2;
using namespace cv;

double medianMat(cv::Mat Input);

int main(int argc, char** argv)
{
		cv::Mat Image(5,5,CV_32F);
		cv::Mat dst(5,5,CV_32F);
		randu(Image, Scalar::all(0), Scalar::all(10));
		std::cout << Image << std::endl;
		int ksize = 3;
		cv::medianBlur(Image, dst, ksize);

		std::cout << dst << std::endl;

	return 0;
}

double medianMat(cv::Mat Input){
//Input = Input.reshape(0,1); // spread Input Mat to single row
std::vector<double> vecFromMat;
Input.copyTo(vecFromMat); // Copy Input Mat to vector vecFromMat
std::nth_element(vecFromMat.begin(), vecFromMat.begin() + vecFromMat.size() / 2, vecFromMat.end());
return vecFromMat[vecFromMat.size() / 2];}
