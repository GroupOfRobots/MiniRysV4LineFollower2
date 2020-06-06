#include "../bcm2835/bcm2835.h"
#include "../motors/l6470constants.h"
#include "../motors/motors.h"
#include "../pid/Pid.h"
#include "../detector/Detector.h"
#include "../data_saver/DataSaver.h"
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
#include <mutex>

#define GPIO_TOF_1 	RPI_V2_GPIO_P1_12
#define GPIO_TOF_2 	RPI_V2_GPIO_P1_16
#define GPIO_TOF_3 	RPI_V2_GPIO_P1_18
#define GPIO_TOF_4 	RPI_V2_GPIO_P1_29
#define GPIO_TOF_5	RPI_V2_GPIO_P1_32
#define GPIO_TOF_6 	RPI_V2_GPIO_P1_31
#define GPIO_TOF_7 	RPI_V2_GPIO_P1_33
#define GPIO_TOF_8 	RPI_V2_GPIO_P1_35
#define GPIO_TOF_9 	RPI_V2_GPIO_P1_36
#define GPIO_TOF_10 RPI_V2_GPIO_P1_37
#define NDEBUG

#ifndef CONTROLLER_H
#define CONTROLLER_H

class Controller {
	private:
		int control_period_;
		std::shared_ptr<Detector> detector_;
		std::shared_ptr<Motors> board_;
		std::shared_ptr<Pid> pid_; 
		std::shared_ptr<DataSaver> data_saver_;
		std::pair<int, int> control_;
		std::vector<cv::Point> line_centers_;
		int image_center_;
		std::mutex mtx_;
		void run_control();
		
	public:
		Controller(double K, double Ti, double Td, int uWorkPoint, int uMin, int uMax, std::shared_ptr<Detector> detector, int control_period = 60000);
		~Controller();
		void run();
		std::map<std::string, int> getProcessData();
};

#endif
