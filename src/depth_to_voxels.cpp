#include <iostream>
#include <ros/package.h>

/* OpenCV 2 */
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.hpp>

using namespace cv;

cv::Mat GenerateKinectImage();
std::vector<cv::Point3d> Generate3DPoints();
void getCameraParameters(cv::Mat intrinsicMat);
void getCameraPose(cv::Mat rVec, cv::Mat tVec);
void getPointDepth(std::vector<cv::Point3d> worldpoints, cv::Mat rVec, cv::Mat tVec, cv::Mat voxeldistances);
//void fillVoxels(cv::Mat Image, std::vector<cv::Point2d> imagePoints, cv::Mat voxeldistances, cv::Mat voxels);


int main()
{
	// Get Sample kinect image
	Mat Image = GenerateKinectImage();

	// Read 3D points
	std::vector<cv::Point3d> objectPoints = Generate3DPoints();
	std::vector<cv::Point2d> imagePoints;
	Mat voxeldistances = Mat(1,objectPoints.size(), cv::DataType<double>::type);

	// Define intrinsic camera parameters
	cv::Mat intrinsicMat(3, 3, cv::DataType<double>::type); // intrinsic matrix
	getCameraParameters(intrinsicMat);

	// Define Camera orientation and position
	cv::Mat rVec(3, 1, cv::DataType<double>::type); // Rotation vector
	cv::Mat tVec(3, 1, cv::DataType<double>::type); // Translation vector in camera frame
	getCameraPose(rVec,tVec);

	// Display some matrices
	std::cout << "intrinsic matrix: "    << std::endl << " " << intrinsicMat << std::endl << std::endl;
	std::cout << "Rotation vector: "    << std::endl << " " << rVec << std::endl << std::endl;
	std::cout << "Translation vector: " << std::endl << " " << tVec << std::endl << std::endl;

	// Get distance to camera
	getPointDepth(objectPoints, rVec, tVec, voxeldistances);

	//Clock in
	double t = (double)getTickCount();

	// Project 3D points onto image plane
	cv::projectPoints(objectPoints, rVec, tVec, intrinsicMat, cv::Mat(), imagePoints);

	// Extract depth measured at imagePoints


	std::cout << "Voxeldistances "    << std::endl << " " << voxeldistances << std::endl << std::endl;

	// Display the projections done
	for (unsigned int i = 0; i < imagePoints.size(); ++i)
	{
		std::cout << "Image point: " << objectPoints[i] << " Projected to " << imagePoints[i] << std::endl;
	}

	// Clock out
	t = ((double)getTickCount() - t)/getTickFrequency();
	std::cout << "Times passed in seconds: " << t << std::endl;

	return 0;
}

void getPointDepth(std::vector<cv::Point3d> worldpoints, cv::Mat rVec, cv::Mat tVec, cv::Mat voxeldistances)
{
	// Get rotation matrix
	cv::Mat R(3, 3, cv::DataType<double>::type);
	Rodrigues(rVec, R);
	std::cout << "Rotation Matrix: "    << std::endl << " " << R << std::endl << std::endl;

	for (int i; i<worldpoints.size(); i++)
	{
		voxeldistances.at<double>(i)  = worldpoints[i].z * R.at<double>(2,2) - tVec.at<double>(2);
	}

}


cv::Mat GenerateKinectImage()
{
	// Create fake kinect image
	cv::Mat M = Mat(480,640,cv::DataType<double>::type);

	// fill it with random values between 1 and 2
	randu(M, Scalar::all(1), Scalar::all(2));

	return M;
}


std::vector<cv::Point3d> Generate3DPoints()
								{
	std::vector<cv::Point3d> points;

	double x, y, z;
	//    for (int i; i<1;i++)
	//    {
	//    	x = 0;
	//    	y = 0;
	//    	z = 0;
	//    points.push_back(cv::Point3d(x, y, z));
	//    }

	x = 0; y = 0; 	z = 0;
	points.push_back(cv::Point3d(x, y, z));

	x = 0; y = 1; 	z = 0;
	points.push_back(cv::Point3d(x, y, z));

	x = 0; y = 0; 	z = 1;
	points.push_back(cv::Point3d(x, y, z));

	x = 1; y = 0; 	z = 0;
	points.push_back(cv::Point3d(x, y, z));

	return points;
								}

void getCameraParameters(cv::Mat intrinsicMat)
{
	intrinsicMat.at<double>(0, 0) = 589.3667;  // 640/2/tand(57/2)
	intrinsicMat.at<double>(1, 0) = 0;
	intrinsicMat.at<double>(2, 0) = 0;

	intrinsicMat.at<double>(0, 1) = 0;
	intrinsicMat.at<double>(1, 1) = 609.2755;  // 480/2/tand(43/2)
	intrinsicMat.at<double>(2, 1) = 0;

	intrinsicMat.at<double>(0, 2) = 319.5;  // 640/2
	intrinsicMat.at<double>(1, 2) = 239.5;  // 240/2
	intrinsicMat.at<double>(2, 2) = 1;
}

void getCameraPose(cv::Mat rVec, cv::Mat tVec)
{
	// Rotation vector in Rodrigues angles
	rVec.at<double>(0) = 0;
	rVec.at<double>(1) = 0;
	rVec.at<double>(2) = 0;

	// Translation vector in Camera frame
	tVec.at<double>(0) = 0;
	tVec.at<double>(1) = 0;
	tVec.at<double>(2) = -5;
}
