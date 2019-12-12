#include <csignal>
#include <math.h>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <ctime>
#include <chrono>
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include "../camera/ContourFinding.h"

class Detector {
	private:
		double roi_up_limit_;
		double roi_down_limit_;
		int acquisition_period_;
		int threshold_;
		double scale_;
		std::shared_ptr<ContourFinding> contour_finder_;
		std::shared_ptr<cv::VideoCapture> clip_capture_;
		cv::Mat frame_;
		std::vector<cv::Point> line_centers_;
		void runDetection();
		
		
	public:
		Detector(double roi_up_limit, double roi_down_limit, double scale, int threshold = 50, int acquisition_period = 60000);
		~Detector();
		std::vector<Point> getLineCenters();
		cv::Mat getFrame();
		int getImageCenter();
		void run();
};

