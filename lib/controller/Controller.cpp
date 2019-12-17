#include "Controller.h"

Controller::Controller(double K, double Ti, double Td, int uWorkPoint, int uMin, int uMax, std::shared_ptr<Detector> detector, int control_period): 
detector_(detector), control_period_(control_period), control_(std::make_pair(0, 0)), image_center_(0){
	
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

		mtx_.lock();
		line_centers_ = detector_->getLineCenters();
		image_center_ = detector_->getImageCenter();
		pid_->setSetPoint(image_center_);
		if(line_centers_.empty()) control_ = pair<int, int>(0, 0);
		else control_ = pid_->calculateControl(line_centers_[0].x);
		if(line_centers_.empty()) std::cout<<" Img center: "<<image_center_<<std::endl;
		else std::cout<<"Line center: "<<line_centers_[0].x<<" Img center: "<<image_center_<<std::endl;
		//board_->setSpeed(-p.first, -p.second); //stops controller when no motors connected
		mtx_.unlock();

		auto end = chrono::steady_clock::now();
		int duration = chrono::duration_cast<chrono::microseconds>(end - start).count();
		if(duration < control_period_) std::this_thread::sleep_for(std::chrono::microseconds(control_period_ - duration));
		else std::cout<<"EXCEEDED REGULATION LOOP TIME BY: "<< duration - control_period_ <<endl;
		auto end1 = chrono::steady_clock::now();
		std::cout<<"Loop time: "<< chrono::duration_cast<chrono::microseconds>(end1 - start1).count()<<endl;
	}
}

std::map<std::string, int> Controller::getProcessData(){
	std::map<std::string, int> process_data;

	mtx_.lock();
	process_data["motor_left"] = control_.first;
	process_data["motor_right"] = control_.second;
	process_data["img_center"] = image_center_;
	if(line_centers_.empty()) process_data["line_center"] = 0;
	else process_data["line_center"] = line_centers_[0].x;
	mtx_.unlock();

	return process_data;
}

void Controller::run(){
	std::thread detector(&Controller::runControl, this);
	detector.detach();
}