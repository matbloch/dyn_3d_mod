/*
 * script.cpp
 *
 *  Created on: 16 mrt. 2015
 *      Author: RGrandia
 */

// Eigen
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

// Configure library
#include <config/config_handler.h>

ConfigHandler conf;

int main(int argc, char** argv)
{
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
	gridsize = 64;   // Must be 2^x
	spacing_in_m = 0.05;
	grid1.setParameters(gridsize, spacing_in_m, intrinsicMat, R1, tVec1);
	grid2.setParameters(gridsize, spacing_in_m, intrinsicMat, R2, tVec2);

	std::cout<< extrinsics_mat << std::endl;
	std::cout<< R_ext << std::endl;
	std::cout<< t_ext << std::endl;
}
