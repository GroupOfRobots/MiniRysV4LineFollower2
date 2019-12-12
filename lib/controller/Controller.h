#include "../bcm2835/bcm2835.h"
#include "../motors/l6470constants.h"
#include "../motors/motors.h"
#include "../pid/Pid.h"
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

class Controller {
	private:
		int control_period_;
		std::shared_ptr<Motors> board_;
		std::shared_ptr<Pid> pid_; 
		void runControl();
		
		
	public:
		Controller(double K, double Ti, double Td, double T, int uWorkPoint, int uMin, int uMax, int control_period = 60000);
		~Controller();
		void run();
};

