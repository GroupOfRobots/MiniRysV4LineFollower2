#include <ctime>
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
using namespace cv;
using namespace std;
using boost::asio::ip::udp;

class UdpJpgFrameStreamer {
	private:
		int port;
		int dataSize;
		boost::asio::io_service io_service;
     	udp::endpoint remote_endpoint;
		udp::socket* socket;
		vector<int> compression_params;
		Mat frame_to_send;
		bool ready_to_send;
		std::mutex mtx; 

		void waitForClient();
		void runStream();
		void uploadFrame();
		
	public:
		UdpJpgFrameStreamer(int port=2024, int dataSize = 64000, int jpegCompressionLevel = 80);
		void run(); 
		void pushFrame(Mat frame); 
};

