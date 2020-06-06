#include "ContourFinding.h"

ContourFinding::ContourFinding(double point_factor_to_start_cutting, double point_factor_to_finish_cutting){
	this->point_factor_to_start_cutting_ = point_factor_to_start_cutting;
	this->point_factor_to_finish_cutting_ = point_factor_to_finish_cutting;
	point_to_start_cutting_ = Point(0,output_frame_.rows*point_factor_to_start_cutting);
	point_to_finish_cutting_ = Point(output_frame_.cols, output_frame_.rows*point_factor_to_finish_cutting);
}

ContourFinding::ContourFinding(Mat frame, double point_factor_to_start_cutting, double point_factor_to_finish_cutting){
	source_frame_ = frame;
	output_frame_ = frame;
	this->point_factor_to_start_cutting_ = point_factor_to_start_cutting;
	this->point_factor_to_finish_cutting_ = point_factor_to_finish_cutting;
	point_to_start_cutting_ = Point(0,output_frame_.rows*point_factor_to_start_cutting);
	point_to_finish_cutting_ = Point(output_frame_.cols, output_frame_.rows*point_factor_to_finish_cutting);
}

void ContourFinding::setFrame(Mat frame){
	source_frame_ = frame;
	output_frame_ = frame;
	contour_.erase(contour_.begin(), contour_.end());
	point_to_start_cutting_ = Point(0, output_frame_.rows*point_factor_to_start_cutting_);
	point_to_finish_cutting_ = Point(output_frame_.cols, output_frame_.rows*point_factor_to_finish_cutting_);
}

Mat ContourFinding::getSourceFrame(){
	return source_frame_;
}

Mat ContourFinding::getOutputFrame(){
	return output_frame_;
}

void ContourFinding::cut_image(){
	Rect r(point_to_start_cutting_, point_to_finish_cutting_);
	output_frame_ = output_frame_(r);
}

void ContourFinding::to_gray_scale(){
	cvtColor(output_frame_, output_frame_, cv::COLOR_BGR2GRAY);
}

void ContourFinding::erode_frame(Mat element){
	erode(output_frame_, output_frame_, element);
}

void ContourFinding::dilate_frame(Mat element){
	dilate(output_frame_, output_frame_, element);
}

void ContourFinding::setThreshold(int threshold){
	this->binarize_threshold_ = threshold;
}

void ContourFinding::threshold_frame(){
	threshold(output_frame_, output_frame_, binarize_threshold_, 255, THRESH_BINARY_INV);
}

vector<Point> ContourFinding::find_centers(){
	vector<Point> centers;
	vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

	double largest_area = 0;
	int largest_contour_index = 0;

	//znalezienie konturów (najbardziej zewnętrznych)
	findContours(output_frame_, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);    

	//jeśli znaleziono jakiś kontur
	if(!contours.empty()) {
		//szukanie największego konturu
		for( int i = 0; i< contours.size(); i++){
       			double a=contourArea(contours[i],false);
       			if(a>largest_area)
				{
       				largest_area=a;
       				largest_contour_index=i;
       			}
    		}

		vector<Point> contour = contours[largest_contour_index];
		this->contour_=contour;

		//wyliczenie momentów figury
		Moments mu = moments(contours[largest_contour_index], false);

		//wyliczenie i zapamiętanie środka cięzkości figury
		centers.push_back(Point(mu.m10/mu.m00, mu.m01/mu.m00));
	}
	
	//jeśli nie znaleziono konturu
	else{
		printf("No contours \n");
	} 

	return centers;
}

vector<Point> ContourFinding::findLineCenters(){
	cut_image();

	to_gray_scale();

	Mat element = getStructuringElement( MORPH_RECT,Size( 2*max_kernel_size_ + 1, 2*max_kernel_size_+1 ),Point( max_kernel_size_, max_kernel_size_));

	erode_frame(element);

	dilate_frame(element);

	threshold_frame();

	return find_centers();
}

Mat ContourFinding::drawPoints(vector<Point> centers){
	Mat frame = source_frame_.clone();
	vector<vector<Point>> contours;
	vector<Point> contour;

	//rysowanie wykrytych środków ciężkosci linii i konturów
	Scalar color(0, 0, 255);
    Scalar colorContour(0, 255, 0);
	
	//przesunięcie konturu
	for( int i = 0; i< this->contour_.size(); i++ )
    {
        contour.push_back(this->contour_[i] + point_to_start_cutting_);
    }
	
	contours.push_back(contour);

	//rysowanie konturu
	if (contour.size() != 0) {
		drawContours(frame, contours, 0, color, 2, 8);
	}

	//rysowanie prostokąta zasięgu
	rectangle( frame, point_to_start_cutting_, point_to_finish_cutting_, colorContour, 0, 8 );

	//rysowanie środka ciężkosci konturu
	if(!centers.empty())
	{
		for(int i=0;i<centers.size();i++)
		{
			//rysowanie wykrytego środka ciężkosci figury
			circle(frame, centers[i]+point_to_start_cutting_, 5, color, -1, 8);
		}
		
		//rysowanie linii
		line(frame, Point(centers[0].x,0), Point(centers[0].x, frame.rows), color,2);

	}

	return frame;
}
		
