#include "Controller.h"

Controller::Controller(double K, double Ti, double Td, int uWorkPoint, int uMin, int uMax, std::shared_ptr<Detector> detector, int control_period): 
detector_(detector), control_period_(control_period) {
	
	if (bcm2835_init() == 0) throw std::runtime_error("Unable to init the bmc2835\n");
	pid_ = std::make_shared<Pid>(K, Ti, Td, static_cast<double>(control_period), uWorkPoint, uMin, uMax);
	board_ = std::make_shared<Motors>(BCM2835_SPI_CS0, GPIO_RESET_OUT);
	board_->setUp();
	board_->resetPosition();
}

Controller::~Controller(){
		//if (globalBoard != nullptr) globalBoard->stop();
		board_->stop();
		std::cout<<"CONTROLLER TERMINATED"<<endl;
}

void Controller::runControl(){
	while(1){
		auto start1 = chrono::steady_clock::now();
		auto start = chrono::steady_clock::now();
		pair<int, int> p; 
		std::vector<cv::Point> centers = detector_->getLineCenters();
		int center = detector_->getImageCenter();
		if(centers.empty()) std::cout<<" Img center: "<<center<<std::endl;
		else std::cout<<"Line center: "<<centers[0].x<<" Img center: "<<center<<std::endl;

		pid_->setSetPoint(center);
		if(centers.empty()) p = pair<int, int>(0, 0);
		else p = pid_->calculateControl(centers[0].x);
		//board_->setSpeed(-p.first, -p.second); //stops controller when no motors connected

		auto end = chrono::steady_clock::now();
		int duration = chrono::duration_cast<chrono::microseconds>(end - start).count();
		if(duration < control_period_) std::this_thread::sleep_for(std::chrono::microseconds(control_period_ - duration));
		else std::cout<<"EXCEEDED REGULATION LOOP TIME BY: "<< duration - control_period_ <<endl;
		auto end1 = chrono::steady_clock::now();
		std::cout<<"Loop time: "<< chrono::duration_cast<chrono::microseconds>(end1 - start1).count()<<endl;
	}
}

void Controller::run(){
	std::thread detector(&Controller::runControl, this);
	detector.detach();
}
