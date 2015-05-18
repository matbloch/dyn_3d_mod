#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <misc/queue.h>


// Eigen
#include <Eigen/Dense>

// OpenCV includes
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

// Boost
#include <boost/thread/thread.hpp>

void provider();
void worker();

Queue<cv::Mat> myqueue;


int main(){
	std::cout << "Start" << std::endl;
	// start threads
    boost::thread_group threads;
    boost::thread *t1 = new boost::thread(provider);
    boost::thread *t2 = new boost::thread(worker);
    threads.add_thread(t1);
    threads.add_thread(t2);

    threads.join_all();

	return 0;

}

void provider(){
	int counter = 1;
	while(counter < 7){
		int gridsize = 2;
		int sz[3] = {gridsize,gridsize,gridsize};
		cv::Mat Voxels = cv::Mat(3,sz, CV_32FC3, cv::Scalar::all(0));
		Voxels.at<float>(0,0,0) = (float)counter;
		myqueue.push(Voxels);
		std::cout << "- provider added:" << Voxels.at<float>(0,0,0) << std::endl;
		usleep(100000);
		counter ++;
	}
}

void worker(){
	cv::Mat output;
	while(true){

//		if(!myqueue.isempty()){
		std::cout << "--- empty state: " << myqueue.isempty() << std::endl;
		output = myqueue.pop();
		std::cout << "--- Worker fetched: " << output.at<float>(0,0,0) << std::endl;

			usleep(500000);
//		}
//		else{
//			std::cout << "Broken" << std::endl;
//			break;
//		}
	}
}
