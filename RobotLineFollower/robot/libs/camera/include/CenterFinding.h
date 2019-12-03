#include <ctime>
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <stdio.h>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

class CenterFinding{
	private:
		Point pointToStartCutting;
		Point pointToFinishCutting;
		Mat sourceFrame;
		Mat outputFrame; 
		int numOfPoints;
		int max_kernel_size = 21;
		int max_kernel_length = 31;
		double scaleFactor = 0.5;
		
		void scaleImage();
		void cutImage();
		void toGrayScale();
		void useBlur();
		void erodeFrame(Mat element);
		void dilateFrame(Mat element);
		void thresholdFrame();
		vector<Point> findCenters();
		
	public:
		CenterFinding(int numOfPixels = 3);
		CenterFinding(Mat frame, int numOfPixels = 3);
		void setSourceFrame(Mat frame);
		void setFrame(Mat frame);
		void setPointsToApproachCutting(Point startPoint, Point endPoint);
		Point getStartPointToApproachCutting();
		Point getEndPointToApproachCutting();
		void setScaleFactor(double scaleFactor);
		double getScaleFactor();
		Mat getSourceFrame();
		Mat getOutputFrame();
		vector<Point> findLineCenters();
		Mat drawPoints(vector<Point> centers);
		
};
