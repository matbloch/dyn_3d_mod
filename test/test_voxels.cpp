/*
 * script.cpp
 *
 *  Created on: 16 mrt. 2015
 *      Author: RGrandia
 */

#include "cv/voxel_grid.h"

	voxelGrid grid;

	using namespace cv;
	void getCameraParameters(cv::Mat intrinsicMat);
	void getCameraPose(cv::Mat R, cv::Mat tVec);

int main(int argc, char** argv)
{
	// Read the kinect image
		cv::Mat kinectimage(480, 640, CV_32F, Scalar(5));

		// Define intrinsic camera parameters
		cv::Mat intrinsicMat(3, 3, CV_32F); // intrinsic matrix
		getCameraParameters(intrinsicMat);

		// Define Camera orientation and position
		cv::Mat R(3, 3, CV_32F);
		cv::Mat tVec(3, 1, CV_32F); // Translation vector in camera frame
		getCameraPose(R,tVec);

		// Set Voxel parameters
		int gridsize = 4;   // Must be 2^x
		float spacing_in_m = 0.02;

		int sz[3] = {gridsize,gridsize,gridsize};
		Mat FilledVoxels(3,sz, CV_32FC1, Scalar::all(0));

		grid.setParameters(gridsize, spacing_in_m, intrinsicMat, R, tVec);
		grid.fillVoxels(kinectimage, FilledVoxels);

		return 0;
}

/* ========================================== *\
 * 		         Camera settings
\* ========================================== */

void getCameraParameters(cv::Mat intrinsicMat)
{
	intrinsicMat.at<float>(0, 0) = 589.3667;  // 640/2/tand(57/2)
	intrinsicMat.at<float>(1, 0) = 0;
	intrinsicMat.at<float>(2, 0) = 0;

	intrinsicMat.at<float>(0, 1) = 0;
	intrinsicMat.at<float>(1, 1) = 609.2755;  // 480/2/tand(43/2)
	intrinsicMat.at<float>(2, 1) = 0;

	intrinsicMat.at<float>(0, 2) = 319.5;  // 640/2
	intrinsicMat.at<float>(1, 2) = 239.5;  // 240/2
	intrinsicMat.at<float>(2, 2) = 1;
}

void getCameraPose(cv::Mat R, cv::Mat tVec)
{
	cv::Mat rVec(3, 1, cv::DataType<float>::type); // Rotation vector
	// Rotation vector in Rodrigues angles
	rVec.at<float>(0) = 0;
	rVec.at<float>(1) = 0;
	rVec.at<float>(2) = 0;
	Rodrigues(rVec, R);

	// Translation vector in Camera frame
	tVec.at<float>(0) = 0;
	tVec.at<float>(1) = 0;
	tVec.at<float>(2) = -5;
}
