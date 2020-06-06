#include "DataSaver.h"

DataSaver::DataSaver(string txt_file_name, string recording_file_name, string path_to_save_data, int frame_width, int frame_height, int frame_interval = 100000) : 
path_to_save_data_(path_to_save_data), frame_height_(frame_height), frame_width_(frame_width) 
{
	frame_interval = int(round(1000000/frame_interval)); //conversion from microseconds to seconds
	video_writer_ = VideoWriter(path_to_save_data + "/" + recording_file_name + ".avi", CV_FOURCC('M','J','P','G'), frame_interval, Size(frame_width, frame_height));
	txt_writer_.open(path_to_save_data + "/" + txt_file_name + ".txt");
	start_ = chrono::steady_clock::now();
}

DataSaver::~DataSaver(){
	video_writer_.release();
	txt_writer_.close();
}

void DataSaver::setFrame(Mat frame){
	if (!frame.empty()) video_writer_.write(frame);
	else txt_writer_ << "Error: couldn't write frame"<<endl;
}

void DataSaver::setDataToTxt(int left_motor, int right_motor, int set_point, int error, int exec_duration){
	auto end = chrono::steady_clock::now();
	double timestamp = chrono::duration_cast<chrono::milliseconds>(end - start_).count();
	txt_writer_ << left_motor << "," << right_motor << "," << set_point << "," << error << "," << exec_duration << "," << timestamp << endl;
}

void DataSaver::setData(Mat frame, int left_motor, int right_motor, int set_point, int error, int exec_duration){
	setFrame(frame);
	setDataToTxt(left_motor, right_motor, set_point, error, exec_duration);
}
