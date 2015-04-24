// Opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <cv/filters.h>
ImageFilters filters;


using namespace std;

int main (int argc, char **argv)
{


    if( argc != 2)
    {
     cout <<" Usage: test_filters {yourImage}" << endl;
     return -1;
    }

    cv::Mat image;
    image = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);   // Read the file


    if(! image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }


    cv::Mat gaussian_filtered;
    cv::Mat bilateral_filtered;
    cv::Mat combined_filtered;

    // prefiltering
    filters.gaussian(image, gaussian_filtered);
    filters.bilateral(image, bilateral_filtered,5);

	cv::imshow("Original image", image);
	cv::waitKey(30000);

    // display filter result
	cv::imshow("Gaussian filter", gaussian_filtered);
	cv::waitKey(30000);

	cv::imshow("Bilater filter", bilateral_filtered);
	cv::waitKey(30000);


  return (0);
}
