#include <iostream>
#include <string>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <fstream>
#include <chrono>
using namespace cv;
using namespace std;

#ifndef DATA_SAVER_H
#define DATA_SAVER_H

class DataSaver {
	private:
		string path_to_save_data_;
		int frame_width_;
		int frame_height_;
		VideoWriter video_writer_;
		ofstream txt_writer_;
		chrono::steady_clock::time_point start_;
		
	public:
		DataSaver(string txt_file_name, string recording_file_name, string path_to_save_data, int frame_width, int frame_height, int frame_interval);
		~DataSaver();
		void setFrame(Mat frame);
		void setDataToTxt(int left_motor, int right_motor, int set_point, int error, int exec_duration);
		void setData(Mat frame, int left_motor, int right_motor, int set_point, int error, int exec_duration);
};

#endif
