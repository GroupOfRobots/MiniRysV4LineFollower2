#include "CenterFinding.h"

CenterFinding::CenterFinding(int numOfPoints){
	this->numOfPoints = numOfPoints;
}

CenterFinding::CenterFinding(Mat frame, int numOfPoints){
	this->numOfPoints = numOfPoints;
	sourceFrame = frame;
	outputFrame = frame;
	pointToStartCutting = Point(0,outputFrame.rows/3);
	pointToFinishCutting = Point(outputFrame.cols, 2*(outputFrame.rows)/3);
}

void CenterFinding::setFrame(Mat frame){
	sourceFrame = frame;
	outputFrame = frame;
	pointToStartCutting = Point(0,outputFrame.rows/3);
	pointToFinishCutting = Point(outputFrame.cols, 2*(outputFrame.rows)/3);
}

Mat CenterFinding::getSourceFrame(){
	return sourceFrame;
}

Mat CenterFinding::getOutputFrame(){
	return outputFrame;
}

Point CenterFinding::getStartPointToApproachCutting(){
	return pointToStartCutting;
}

Point CenterFinding::getEndPointToApproachCutting(){
	return pointToFinishCutting;
}

void CenterFinding::setPointsToApproachCutting(Point startPoint, Point endPoint){
	pointToStartCutting = startPoint;
	pointToFinishCutting = endPoint;
}

void CenterFinding::setScaleFactor(double scaleFactor){
	this->scaleFactor = scaleFactor;
}

double CenterFinding::getScaleFactor(){
	return this->scaleFactor;
}

void CenterFinding::scaleImage(){
	resize(outputFrame, outputFrame, Size(0,0), scaleFactor, scaleFactor, 2); //2->INTER_AREA_INTERPOLATION
	resize(sourceFrame, sourceFrame, Size(0,0), scaleFactor, scaleFactor, 2); //2->INTER_AREA_INTERPOLATION
	pointToStartCutting = Point(0,outputFrame.rows/3);
	pointToFinishCutting = Point(outputFrame.cols, 2*(outputFrame.rows)/3);	
}

void CenterFinding::cutImage(){
	Rect r(0, 0, outputFrame.cols, 2*(outputFrame.rows)/3);
	outputFrame = outputFrame(r);
}

void CenterFinding::toGrayScale(){
	cvtColor(outputFrame, outputFrame, cv::COLOR_RGB2GRAY);
}

void CenterFinding::useBlur(){
	medianBlur (outputFrame, outputFrame, max_kernel_length);
}

void CenterFinding::erodeFrame(Mat element){
	erode(outputFrame, outputFrame, element );
}

void CenterFinding::dilateFrame(Mat element){
	dilate(outputFrame, outputFrame, element);
}

void CenterFinding::thresholdFrame(){
	threshold(outputFrame, outputFrame, 127, 255, THRESH_BINARY_INV);
}

vector<Point> CenterFinding::findCenters(){
	vector<Point> centers;
	int spaceWidth = outputFrame.rows/(numOfPoints+1);
	int sum = 0;
	int center = 0;
	int whitePixelCounter = 0;

	for(int i = 1; i <= numOfPoints; i++)
	{
		int row = i*spaceWidth;
		for(int j = 0; j < outputFrame.cols; j++)
		{
			if(int(outputFrame.at<uchar>(row,j)) == 255)
			{
				sum+=j;
				whitePixelCounter++;
			}
		}

		if(whitePixelCounter != 0)
		{
			center = sum/whitePixelCounter;
			centers.push_back(Point(center,row));
		}
	
		sum=0;
		whitePixelCounter=0;
	}

	return centers;
}

vector<Point> CenterFinding::findLineCenters(){
	scaleImage();
	
	//unneccesary here
	//cutImage();

	toGrayScale();

	//useBlur();//commented, because might be unuseful

	Mat element = getStructuringElement( MORPH_RECT,Size( 2*max_kernel_size + 1, 2*max_kernel_size+1 ),Point( max_kernel_size, max_kernel_size ));

	erodeFrame(element);

	dilateFrame(element);

	thresholdFrame();

	return findCenters();
}

Mat CenterFinding::drawPoints(vector<Point> centers){
	Mat frame = sourceFrame.clone();

	//rysowanie wykrytych środków ciężkosci linii
	Scalar colorPoint(0, 0, 255);
	Scalar colorLine(0, 255, 0);

	double mean = 0;

	//rysowanie linii i jej środka ciężkosci
	for(int i=0;i<centers.size();i++)
	{
		line(frame,Point(0,centers[i].y),Point(frame.cols,centers[i].y),colorLine);
    	circle(frame, centers[i], 5, colorPoint, -1, 8);
		mean += centers[i].x/centers.size();
    }
	
	//rysowanie linii
    line(frame, Point(mean,0), Point(mean,frame.rows), colorPoint,2);

	return frame;
}
		
