#include <iostream>
#include <ros/package.h>

/* OpenCV 2 */
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.hpp>

struct Vox_param {
	int gridsize;
	float spacing_in_m;
} ;

using namespace cv;
void getCameraParameters(cv::Mat intrinsicMat);
void getCameraPose(cv::Mat R, cv::Mat tVec);
void calcVoxel_Depth_Pixels(cv::Mat Voxels, Vox_param Voxelparameters, cv::Mat R, cv::Mat tVec, cv::Mat intrinsicMat);
void fillVoxels(cv::Mat Voxels, Vox_param Voxelparameters, cv::Mat image, cv::Mat FilledVoxels);
float TSDF(float d_in);

int main()
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

	// Display some matrices
	std::cout << "intrinsic matrix: "    << std::endl << " " << intrinsicMat << std::endl << std::endl;
	std::cout << "Rotation matrix: "    << std::endl << " " << R << std::endl << std::endl;
	std::cout << "Translation vector: " << std::endl << " " << tVec << std::endl << std::endl;

	// Set Voxel parameters
	Vox_param Voxelparameters;
	Voxelparameters.gridsize = 32;   // Must be 2^x
	Voxelparameters.spacing_in_m = 0.05;

	// Create voxels
		// 3d matrix with 3 channel floatingpoints.
		// Access individual element by:  pix = Voxels.at<cv::Vec3f>(i,j,k)  and  pix[channel]
	int sz[3] = {Voxelparameters.gridsize,Voxelparameters.gridsize,Voxelparameters.gridsize};
	Mat Voxels(3,sz, CV_32FC3, Scalar::all(0));
	Mat FilledVoxels(3,sz, CV_32FC1, Scalar::all(0));

	// Get depth and pixel values at each Voxel
	calcVoxel_Depth_Pixels(Voxels, Voxelparameters, R, tVec, intrinsicMat);

	//Clock in
	float t = (float)getTickCount();

	// Implement the TSDF
	fillVoxels(Voxels, Voxelparameters, kinectimage, FilledVoxels);

	// Clock out
	t = ((float)getTickCount() - t)/getTickFrequency();
	std::cout << "Times passed in seconds: " << std::setprecision(5) << t << std::endl;

	return 0;
}


/* ========================================== *\
 * 	          Distance function
\* ========================================== */

float TSDF(float d_in)
{
//	         |-| d_margin
//	1 -----
//	       \
//	        \
//	         \
//	          \
//	-1         ----------
//	         |0 ->+      |d_truncate

	// All distances in m
	float d_margin = 0.01;
	float d_truncate = 0.05;

	if (d_in> d_truncate){
		return NAN;
	}
	else if (d_in<-d_margin){
		return 1.0;
	}
	else if (d_in>d_margin){
		return -1.0;
	}
	else {
		return -d_in/d_margin;
	}
}

/* ========================================== *\
 * 	          Fill voxel grid
\* ========================================== */

void fillVoxels(cv::Mat Voxels, Vox_param Voxelparameters, cv::Mat image, cv::Mat FilledVoxels)
{
	float d_meas;

	// Loop over the whole voxel grid
		for (int i = 0; i < Voxelparameters.gridsize; i++)
		  for (int j = 0; j < Voxelparameters.gridsize; j++)
		    for (int k = 0; k < Voxelparameters.gridsize; k++)
		    {
		    	// Get measured depth at pixel value:   image(y,x)
		    	d_meas = image.at<float>( (int)Voxels.at<cv::Vec3f>(i,j,k)[1] ,  (int)Voxels.at<cv::Vec3f>(i,j,k)[0]  );

		    	// Evaluate the TSDF with the depth difference and store the result
		    	FilledVoxels.at<float>(i,j,k) = TSDF(Voxels.at<cv::Vec3f>(i,j,k)[2] - d_meas );

//				std::cout << "Distance: " << d_meas   << std::endl;
//				std::cout << "TSDF: " << FilledVoxels.at<float>(i,j,k) << std::endl << std::endl;
		    }
}


/* ========================================== *\
 * 	          Configure voxel grid
\* ========================================== */

void calcVoxel_Depth_Pixels(cv::Mat Voxels, Vox_param Voxelparameters, cv::Mat R, cv::Mat tVec, cv::Mat intrinsicMat)
{
	cv::Mat pointin3d(3, 1, CV_32F);
	cv::Mat camframe(3, 1, CV_32F);
	cv::Mat pix(3, 1, CV_32F);
	float units[Voxelparameters.gridsize]; // [meters]

	// Precalculate an array for the voxel positions in space in [meters]
	for (int n = 0; n<Voxelparameters.gridsize; n++)
	{
		units[n] = Voxelparameters.spacing_in_m*((n+1)-(Voxelparameters.gridsize+1)/2.0);
	}

	// Loop over the whole voxel grid
	for (int i = 0; i < Voxelparameters.gridsize; i++)
	  for (int j = 0; j < Voxelparameters.gridsize; j++)
	    for (int k = 0; k < Voxelparameters.gridsize; k++)
	    {
	    	// Convert Voxel index to 3d point
	    	pointin3d.at<float>(0) = units[i];
	    	pointin3d.at<float>(1) = units[j];
	    	pointin3d.at<float>(2) = units[k];

	    	camframe = R*pointin3d - tVec;
	    	// Assign z-value in camera frame
	    	Voxels.at<cv::Vec3f>(i,j,k)[2] = camframe.at<float>(2);

	    	// Convert to pixels
	    	pix = intrinsicMat*camframe;

	    	// Assign pixel values
			Voxels.at<cv::Vec3f>(i,j,k)[0] = pix.at<float>(0)/pix.at<float>(2);
			Voxels.at<cv::Vec3f>(i,j,k)[1] = pix.at<float>(1)/pix.at<float>(2);
	    }
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
