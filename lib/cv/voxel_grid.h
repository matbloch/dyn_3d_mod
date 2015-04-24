#include <iostream>
#include <ros/package.h>

/* OpenCV 2 */
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.hpp>
#include <math.h>

using namespace cv;

class voxelGrid {

  public:
		// Constructor setting all private parameters and projecting the voxels into the camera image
		voxelGrid(int,float,Mat,Mat,Mat);

		// Main function taking a 480X640 depth image and returning the filled in voxels in second input
		void fillVoxels(Mat, Mat);

		// input the voxels according to:
		//		int sz[3] = {gridsize,gridsize,gridsize};
		//	    Mat FilledVoxels(3,sz, CV_32FC1, Scalar::all(0));

  private:
		/* Parameters */
		int gridsize;
		float spacing_in_m;
		std::vector<float> units; // [meters]

		float d_margin;
		float d_truncate;

		Mat intrinsicMat;     // (3, 3, CV_32F)
		Mat R;			      // (3, 3, CV_32F)
		Mat tVec;			  // (3, 1, CV_32F)
		Mat Voxels;           // Voxels as 3 channel mat, [xpix, ypix, z-value]

		/* Functions */
		float TSDF(float);
		void calcVoxel_Depth_Pixels();
};




/* ========================================== *\
 * 	          Constructor function
\* ========================================== */

voxelGrid::voxelGrid(int a_gridsize, float a_spacing_in_m, Mat a_intrinsicMat, Mat a_R, Mat a_tVec) {
	// Set private parameters
	gridsize = a_gridsize;
	spacing_in_m = a_spacing_in_m;
	intrinsicMat = a_intrinsicMat;
	R = a_R;
	tVec = a_tVec;

	// TSDF parameters
	d_margin = 0.01;
	d_truncate = 0.05;

	// Calculate a vector for the voxel positions in space in [meters]
	units(std::vector<float> (gridsize));
	for (int n = 0; n<gridsize; n++)
	{
		units[n] = spacing_in_m*((n+1)-(gridsize+1)/2.0);
	}

	int sz[3] = {gridsize,gridsize,gridsize};
	Mat Voxels(3,sz, CV_32FC3, Scalar::all(0));

	// Project voxels into image plane and create Mat Voxels
	calcVoxel_Depth_Pixels();
}


/* ========================================== *\
 * 	          Configure voxel grid
\* ========================================== */

void voxelGrid::calcVoxel_Depth_Pixels(){
	// Some temporary variables
	cv::Mat pointin3d(3, 1, CV_32F);
	cv::Mat camframe(3, 1, CV_32F);
	cv::Mat pix(3, 1, CV_32F);

	// Loop over the whole voxel grid
	for (int i = 0; i < gridsize; i++)
	  for (int j = 0; j < gridsize; j++)
	    for (int k = 0; k < gridsize; k++)
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

			// Check if pixels are within image
			if (Voxels.at<cv::Vec3f>(i,j,k)[0]>=640 || Voxels.at<cv::Vec3f>(i,j,k)[0]<0)
				Voxels.at<cv::Vec3f>(i,j,k)[0] = NAN;

			if (Voxels.at<cv::Vec3f>(i,j,k)[1]>=480 || Voxels.at<cv::Vec3f>(i,j,k)[1]<0)
				Voxels.at<cv::Vec3f>(i,j,k)[1] = NAN;
	    }
}

/* ========================================== *\
 * 	          Fill voxel grid
\* ========================================== */

void voxelGrid::fillVoxels(Mat image, Mat FilledVoxels)
{
	// Temporary variable
	float d_meas;

	// Loop over the whole voxel grid
		for (int i = 0; i < gridsize; i++)
		  for (int j = 0; j < gridsize; j++)
		    for (int k = 0; k < gridsize; k++)
		    {
		    	// If pixel is not in image return nan
		    	if (isnan(Voxels.at<cv::Vec3f>(i,j,k)[0]) || isnan(Voxels.at<cv::Vec3f>(i,j,k)[1])){
		    		FilledVoxels.at<float>(i,j,k) = NAN;
		    	}

		    	else{
		    	// Get measured depth at pixel value:   image(y,x)
		    	d_meas = image.at<float>( (int)Voxels.at<cv::Vec3f>(i,j,k)[1] ,  (int)Voxels.at<cv::Vec3f>(i,j,k)[0]  );

		    	// Evaluate the TSDF with the depth difference and store the result
		        FilledVoxels.at<float>(i,j,k) = TSDF(Voxels.at<cv::Vec3f>(i,j,k)[2] - d_meas );
		    	}
		    }
}


/* ========================================== *\
 * 	          Distance function
\* ========================================== */

float voxelGrid::TSDF(float d_in)
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


