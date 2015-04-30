#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <misc/queue.h>

// Boost
#include <boost/thread/thread.hpp>

void provider();
void worker();

Queue<int> myqueue;


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
		myqueue.push(counter);
		std::cout << "- provider added:" << counter << std::endl;
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
