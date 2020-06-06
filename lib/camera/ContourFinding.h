#include <ctime>
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <stdio.h>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

#ifndef CONTOUR_FINDING_H
#define CONTOUR_FINDING_H

class ContourFinding{
	private:
		double point_factor_to_start_cutting_;
		double point_factor_to_finish_cutting_;
		Point point_to_start_cutting_;
		Point point_to_finish_cutting_;
		Mat source_frame_;
		Mat output_frame_; 
		int max_kernel_size_ = 21;
		int binarize_threshold_ = 127;
		vector<Point> contour_;
		
		void cut_image();
		void to_gray_scale();
		void erode_frame(Mat element);
		void dilate_frame(Mat element);
		void threshold_frame();
		vector<Point> find_centers();
		
	public:
		ContourFinding(double pointFactorToStartCutting, double pointFactorToFinishCutting);
		ContourFinding(Mat frame, double pointFactorToStartCutting, double pointFactorToFinishCutting);
		void setFrame(Mat frame);
		void setThreshold(int threshold);
		Mat getSourceFrame();
		Mat getOutputFrame();
		vector<Point> findLineCenters();
		Mat drawPoints(vector<Point> centers);		
};

#endif
