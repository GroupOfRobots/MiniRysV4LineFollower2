#include "Detector.h"

Detector::Detector(double roi_up_limit, double roi_down_limit, double scale, int threshold, int acquisition_period):
roi_up_limit_(roi_up_limit), roi_down_limit_(roi_down_limit), acquisition_period_(acquisition_period), threshold_(threshold), scale_(scale){

	clip_capture_ = std::make_shared<cv::VideoCapture>(0);
	auto height = clip_capture_->get(CV_CAP_PROP_FRAME_HEIGHT);
	auto width = clip_capture_->get(CV_CAP_PROP_FRAME_WIDTH);
	clip_capture_->set(CV_CAP_PROP_CONVERT_RGB, false);
	clip_capture_->set(CV_CAP_PROP_FRAME_HEIGHT, static_cast<int>(height*scale));
  	clip_capture_->set(CV_CAP_PROP_FRAME_WIDTH, static_cast<int>(width*scale));
	if (!clip_capture_->isOpened()) throw std::runtime_error("Could not open reference to clip\n");
	contour_finder_ = std::make_shared<ContourFinding>(roi_up_limit,roi_down_limit);
	//contour_finder_->setScaleFactor(scale);
	contour_finder_->setThreshold(threshold);
}

Detector::~Detector(){
	clip_capture_->release();
}

void Detector::runDetection(){
	while(1){
		//auto start1 = chrono::steady_clock::now();
		auto start = chrono::steady_clock::now();
		int duration;
		cv::Mat src;
		clip_capture_->read(src);
		if (src.empty() || src.cols == -1 || src.rows == -1)
		{
		    	std::cout<<"No image data from clip\n";
		    	break;
		}

		else
		{	
			mtx_.lock();
			contour_finder_->setFrame(src);
			line_centers_ = contour_finder_->findLineCenters();
			frame_ = contour_finder_->drawPoints(line_centers_);
			mtx_.unlock();
			auto end = chrono::steady_clock::now();
			duration = chrono::duration_cast<chrono::microseconds>(end - start).count();
     	}

		if(duration < acquisition_period_) std::this_thread::sleep_for(std::chrono::microseconds(acquisition_period_ - duration));
		//else std::cout<<"EXCEEDED REGULATION LOOP TIME BY: "<< duration - acquisition_period_ <<endl;
		//auto end1 = chrono::steady_clock::now();
		//std::cout<<"Loop time: "<< chrono::duration_cast<chrono::microseconds>(end1 - start1).count()<<endl;
		//std::cout<<"Img center: "<< round(contour_finder_->getSourceFrame().cols/2)<<std::endl;
	}
}

std::vector<Point> Detector::getLineCenters(){
	mtx_.lock();
	std::vector<Point> line_centers = line_centers_;
	mtx_.unlock();
	return line_centers;
}

cv::Mat Detector::getFrame(){
	mtx_.lock();
	cv::Mat frame = frame_;
	mtx_.unlock();
	return frame;
}

int Detector::getImageCenter(){
	mtx_.lock();
	int center = round(contour_finder_->getSourceFrame().cols/2);
	mtx_.unlock();
	return center;
}

void Detector::run(){
	std::thread detector(&Detector::runDetection, this);
	detector.detach();
}
