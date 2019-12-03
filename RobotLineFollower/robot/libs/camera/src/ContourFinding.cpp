#include "ContourFinding.h"

ContourFinding::ContourFinding(double pointFactorToStartCutting, double pointFactorToFinishCutting){
	this->pointFactorToStartCutting = pointFactorToStartCutting;
	this->pointFactorToFinishCutting = pointFactorToFinishCutting;
	pointToStartCutting = Point(0,outputFrame.rows*pointFactorToStartCutting);
	pointToFinishCutting = Point(outputFrame.cols, outputFrame.rows*pointFactorToFinishCutting);
}

ContourFinding::ContourFinding(Mat frame, double pointFactorToStartCutting, double pointFactorToFinishCutting){
	sourceFrame = frame;
	outputFrame = frame;
	this->pointFactorToStartCutting = pointFactorToStartCutting;
	this->pointFactorToFinishCutting = pointFactorToFinishCutting;
	pointToStartCutting = Point(0,outputFrame.rows*pointFactorToStartCutting);
	pointToFinishCutting = Point(outputFrame.cols, outputFrame.rows*pointFactorToFinishCutting);
}

void ContourFinding::setFrame(Mat frame){
	sourceFrame = frame;
	outputFrame = frame;
	contour.erase(contour.begin(),contour.end());
	hierarchy.erase(hierarchy.begin(),hierarchy.end());
	pointToStartCutting = Point(0,outputFrame.rows*pointFactorToStartCutting);
	pointToFinishCutting = Point(outputFrame.cols, outputFrame.rows*pointFactorToFinishCutting);
}

Mat ContourFinding::getSourceFrame(){
	return sourceFrame;
}

Mat ContourFinding::getOutputFrame(){
	return outputFrame;
}

void ContourFinding::setScaleFactor(double scaleFactor){
	this->scaleFactor = scaleFactor;
}

double ContourFinding::getScaleFactor(){
	return this->scaleFactor;
}

//przeskalowanie kaltki
void ContourFinding::scaleImage(){
	resize(outputFrame, outputFrame, Size(0,0), scaleFactor, scaleFactor, 2); //2->INTER_AREA_INTERPOLATION
	resize(sourceFrame, sourceFrame, Size(0,0), scaleFactor, scaleFactor, 2); //2->INTER_AREA_INTERPOLATION
	pointToStartCutting = Point(0,outputFrame.rows*pointFactorToStartCutting);
	pointToFinishCutting = Point(outputFrame.cols, outputFrame.rows*pointFactorToFinishCutting);	
}

void ContourFinding::cutImage(){
	Rect r(pointToStartCutting, pointToFinishCutting);
	outputFrame = outputFrame(r);
}

void ContourFinding::toGrayScale(){
	cvtColor(outputFrame, outputFrame, cv::COLOR_RGB2GRAY);
}

void ContourFinding::useBlur(){
	medianBlur (outputFrame, outputFrame, max_kernel_length);
}

void ContourFinding::erodeFrame(Mat element){
	erode(outputFrame, outputFrame, element );
}

void ContourFinding::dilateFrame(Mat element){
	dilate(outputFrame, outputFrame, element);
}

void ContourFinding::setThreshold(int threshold){
	this->binarize_threshold = threshold;
}

void ContourFinding::thresholdFrame(){
	threshold(outputFrame, outputFrame, binarize_threshold, 255, THRESH_BINARY_INV);
}

vector<Point> ContourFinding::findCenters(){
	vector<Point> centers;
	vector<vector<Point>> contours;
    	vector<Vec4i> hierarchy;

	double largest_area = 0;
	int largest_contour_index = 0;

	//znalezienie konturów (najbardziej zewnętrznych)
	findContours(outputFrame, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);    

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
		this->contour=contour;
		this->hierarchy = hierarchy;
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
	scaleImage();

	cutImage();

	toGrayScale();

	//useBlur();//commented, because might be unuseful

	Mat element = getStructuringElement( MORPH_RECT,Size( 2*max_kernel_size + 1, 2*max_kernel_size+1 ),Point( max_kernel_size, max_kernel_size ));

	erodeFrame(element);

	dilateFrame(element);

	thresholdFrame();

	return findCenters();
}

Mat ContourFinding::drawPoints(vector<Point> centers){
	Mat frame = sourceFrame.clone();
	vector<vector<Point>> contours;
	vector<Point> contour;

	//rysowanie wykrytych środków ciężkosci linii i konturów
	Scalar color(0, 0, 255);
    Scalar colorContour(0, 255, 0);
	
	//przesunięcie konturu
	for( int i = 0; i< this->contour.size(); i++ )
    {
        contour.push_back(this->contour[i] + pointToStartCutting);
    }
	
	contours.push_back(contour);
	//rysowanie konturu
	if (contour.size() != 0) {
		drawContours(frame, contours, 0, color, 2, 8);
	}

	//rysowanie prostokąta zasięgu
	rectangle( frame, pointToStartCutting, pointToFinishCutting, colorContour, 0, 8 );

	//rysowanie środka ciężkosci konturu
	if(!centers.empty())
	{
		for(int i=0;i<centers.size();i++)
		{
			//rysowanie wykrytego środka ciężkosci figury
			circle(frame, centers[i]+pointToStartCutting, 5, color, -1, 8);
		}
		
		//rysowanie linii
		line(frame, Point(centers[0].x,0), Point(centers[0].x, frame.rows), color,2);

	}

	return frame;
}
		
