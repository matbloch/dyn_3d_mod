#include <iostream>
#include <string>


class ImageFilters
{
   public:


		/* Bilateral filtering */
		static void bilateral(cv::Mat in, cv::Mat out);

		/* Gaussian filtering */
		static void gaussian(cv::Mat in, cv::Mat out);

		ImageFilters(void);  // This is the constructor

   private:

};


// constructor
ImageFilters::ImageFilters( void )
{

}

void ImageFilters::bilateral(cv::Mat in, cv::Mat out)
{
	int i = 5;
	cv::bilateralFilter ( in, out, i, i*2, i/2 );
}

void ImageFilters::gaussian(cv::Mat in, cv::Mat out)
{
	// apply gaussian blur with kernel size 11
	cv::GaussianBlur( in, out, cv::Size( 11, 11 ), 0, 0 );
}
