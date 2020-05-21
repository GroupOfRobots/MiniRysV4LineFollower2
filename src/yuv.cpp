#include "opencv2/opencv.hpp"
#include <chrono>
#include <thread>
#include <iostream>

using namespace std;
using namespace cv;
using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds

int main(){
  std::cout<<"OK-1";
  // Create a VideoCapture object and use camera to capture the video
  VideoCapture cap(0); 
  std::cout<<"OK0";
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
  cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
  cap.set(CV_CAP_PROP_CONVERT_RGB, false);
  std::cout<<"OK1";
  // Check if camera opened successfully
  if(!cap.isOpened())
  {
    cout << "Error opening video stream" << endl; 
    return -1; 
  } 
  std::cout<<"OK2";
  // Default resolution of the frame is obtained.The default resolution is system dependent. 
  int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH); 
  int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
  std::cout<<frame_width;
  std::cout<<frame_height; 
  std::cout<<"OK3";
  // Define the codec and create VideoWriter object.The output is stored in 'outcpp.avi' file. 
  VideoWriter video("/home/ubuntu/outcpp.avi",-1,10, Size(frame_width,frame_height), false); 
  while(1)
  { 
    std::cout<<"p"<<std::endl;
    Mat frame; 
    
    // Capture frame-by-frame 
    cap >> frame;
 
    // If the frame is empty, break immediately
    if (frame.empty())
      break;
    
    // Write the frame into the file 'outcpp.avi'
    std::vector<cv::Mat> planes(3);
    cv::split(frame, planes);
	  cv::Mat Y = planes[0];
    imwrite( "/home/ubuntu/yuv_test.jpg", Y );
    sleep_for(seconds(1));

 
    // Press  ESC on keyboard to  exit
    char c = (char)waitKey(1);
    if( c == 27 ) 
      break;
  }

  // When everything done, release the video capture and write object
  cap.release();
  video.release();

  // Closes all the windows
  destroyAllWindows();
  return 0;
}
