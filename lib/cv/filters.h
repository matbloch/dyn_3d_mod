#include <iostream>
#include <string>


class ImageFilters
{
   public:


		/* Bilateral filtering */
		static void bilateral(cv::Mat in, cv::Mat &out, int window_size, float depth_sigma);

		/* Gaussian filtering */
		static void gaussian(cv::Mat in, cv::Mat &out, int filter_size, int filter_order);

		void replaceValue(cv::Mat &img, float search, float replace);

		ImageFilters(void);

   private:

};


// constructor
ImageFilters::ImageFilters( void )
{

}

void ImageFilters::bilateral(cv::Mat in, cv::Mat &out, int window_size = 5, float depth_sigma = 0.03)
{
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

void ImageFilters::replaceValue(cv::Mat &img, float search, float replace)
{
	for(int i=0; i<img.rows; i++)
		for(int j=0; j<img.cols; j++)
			if(img.at<float>(i,j) == search)
				img.at<float>(i,j) = replace;
}
