#include <sensor_msgs/image_encodings.h>

#include <iostream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <atomic>

// Eigen
#include <Eigen/Dense>

// OpenCV includes
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

// our own libraries
#include <config/config_handler.h>
#include <cv/filters.h>
#include "cv/voxel_grid.h"
#include "misc/colio.h"
#include "definitions.h"

// time space tree library
#include "tree/TStree.hpp"

// object handlers
ConfigHandler conf;
ImageFilters filters;
voxelGrid grid1;
voxelGrid grid2;

// globals
cv::Mat intrinsicMat(3, 3, CV_32F); // intrinsic matrix
cv::Mat R1(3, 3, CV_32F);    // Rotation vector
cv::Mat tVec1(3, 1, CV_32F); // Translation vector in camera frame
cv::Mat R2(3, 3, CV_32F);    // Rotation vector
cv::Mat tVec2(3, 1, CV_32F); // Translation vector in camera frame
cv::Mat FilledVoxels1;
cv::Mat FilledVoxels2;
cv::Mat FusedVoxels;

// grid status
static int MAX_DIM = 5;
static int GRID_SIZE = (int)pow(2,MAX_DIM);
static float spacing_in_m = 0.3;
float max_v[3] = {(float)(GRID_SIZE-1), (float)(GRID_SIZE-1), (float)(GRID_SIZE-1)};
float min_v[3] = {0, 0, 0};

// Time Space Tree
TStree tstree((GRID_SIZE-1)*spacing_in_m, MAX_DIM);
int t = 0;


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



int main(int argc, char** argv)
{
	cv::Mat filtered1;
	cv::Mat filtered2;

	cv::FileStorage storage("test/img1.yml", cv::FileStorage::READ);
	storage["img"] >> filtered1;
	storage.release();
	cv::FileStorage storage2("test/img2.yml", cv::FileStorage::READ);
	storage2["img"] >> filtered2;
	storage2.release();

	/* ========================================== *\
	* 		1. Setup voxel struture
	\* ========================================== */


	// Set up the voxel grid objects for both cameras
	getCameraPose(R1,tVec1);

	// get extrinsics
	Eigen::Matrix4f extrinsics;
	conf.getOptionMatrix("camera_parameters.extrinsics", extrinsics);
	cv::Mat extrinsics_mat(4,4, CV_32F);
	cv::eigen2cv(extrinsics, extrinsics_mat);
	cv::Mat R_ext = extrinsics_mat(cv::Range(0,3), cv::Range(0,3));
	cv::Mat t_ext = extrinsics_mat(cv::Range(0,3), cv::Range(3,4));

	// get intrinsics
	Eigen::Matrix3f intrinsics_eig;
	conf.getOptionMatrix("camera_parameters.intrinsics", intrinsics_eig);
	cv::eigen2cv(intrinsics_eig, intrinsicMat);

	// Set up the voxel grid objects for both cameras
	getCameraPose(R1,tVec1);
	R2 = R_ext*R1;
	tVec2 = t_ext + R_ext*tVec1;

	grid1.setParameters(GRID_SIZE, spacing_in_m, intrinsicMat, R1, tVec1);
	grid2.setParameters(GRID_SIZE, spacing_in_m, intrinsicMat, R2, tVec2);

	// Setup voxel structure to load the TSDF in
	int sz[3] = {GRID_SIZE,GRID_SIZE,GRID_SIZE};
	FilledVoxels1 = Mat(3,sz, CV_32FC1, Scalar::all(0));
	FilledVoxels2 = Mat(3,sz, CV_32FC1, Scalar::all(0));
	FusedVoxels = Mat(3,sz, CV_32FC1, Scalar::all(0));


	/* ========================================== *\
	* 		2. Voxel grid
	\* ========================================== */


	// get extrinsics
	//    Eigen::Matrix4f extrinsics;
	//	conf.getOptionMatrix("camera_parameters.extrinsics", extrinsics);

	grid1.fillVoxels(filtered1, FilledVoxels1);
	grid1.fillVoxels(filtered2, FilledVoxels2);


	//ROS_INFO("Voxels filled");

	/* ========================================== *\
	* 		3. Fusion
	\* ========================================== */

	//FusedVoxels = 0.5*(FilledVoxels1 + FilledVoxels2);

	/* ========================================== *\
	* 		4. Octree integration
	\* ========================================== */


	tstree.insert(FilledVoxels1, t);
	t++;
	tstree.insert(FilledVoxels2, t);
	t++;
	tstree.print_timespacetree();


cv::waitKey(0);

}
