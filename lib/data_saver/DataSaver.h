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
		string pathToSaveData;
		int frameWidth;
		int frameHeight;
		VideoWriter videoWriter;
		ofstream txtWriter;
		chrono::steady_clock::time_point start;
		
	public:
		DataSaver(string txtFileName, string recordingFileName, string pathToSaveData, int frameWidth, int frameHeight, int frameInterval);
		~DataSaver();
		void setFrame(Mat frame);
		void setDataToTxt(int leftMotor, int rightMotor, int setPoint, int error, int exec_duration);
		void setData(Mat frame, int leftMotor, int rightMotor, int setPoint, int error, int exec_duration);
};

#endif
