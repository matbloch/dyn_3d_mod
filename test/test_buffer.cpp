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

		cv::Mat Z = cv::Mat::zeros(1,2, CV_8UC1);
		Z.at<int>(0,0) = counter;
		myqueue.push(Z);
		std::cout << "- provider added:" << Z << std::endl;
		usleep(1000000);
		counter ++;
	}
}

void worker(){
	while(true){
		std::cout << "--- Worker fetched: " << myqueue.pop() << std::endl;
		usleep(2000000);
	}
}
