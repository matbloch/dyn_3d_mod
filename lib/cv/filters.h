#include <iostream>
#include <string>


class ImageFilters
{
   public:


		/* Bilateral filtering */
		static void bilateral(cv::Mat in, cv::Mat &out, int window_size);

		/* Gaussian filtering */
		static void gaussian(cv::Mat in, cv::Mat &out, int filter_size, int filter_order);

		ImageFilters(void);

   private:

};


// constructor
ImageFilters::ImageFilters( void )
{

}

void ImageFilters::bilateral(cv::Mat in, cv::Mat &out, int window_size = 5)
{

    const double depth_sigma = 0.03;
    const double space_sigma = 4.5;  // in pixels

	cv::bilateralFilter ( in, out, -1, depth_sigma, window_size );
}

void ImageFilters::gaussian(cv::Mat in, cv::Mat &out, int filter_size = 11, int filter_order = 1)
{
	for(int i=1;i<=filter_order;i++){
		if(i==1){
			cv::GaussianBlur( in, out, cv::Size( filter_size, filter_size ), 0, 0 );
		}else{
			cv::GaussianBlur( in, out, cv::Size( filter_size, filter_size ), 0, 0 );
		}
	}
}
