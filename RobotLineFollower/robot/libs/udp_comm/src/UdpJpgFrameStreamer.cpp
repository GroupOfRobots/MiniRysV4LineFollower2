#include "UdpJpgFrameStreamer.h"

UdpJpgFrameStreamer::UdpJpgFrameStreamer(int port, int dataSize, int jpegCompressionLevel){
	this->dataSize = dataSize;
	this->port = port;
	socket = new udp::socket(io_service, udp::endpoint(udp::v4(), port));
	compression_params.push_back(1);//CV_IMWRITE_JPEG_QUALITY
	compression_params.push_back(jpegCompressionLevel);
	ready_to_send = false;
}

void UdpJpgFrameStreamer::waitForClient(){
	try{
		boost::array<char, 1> recv_buf;
		boost::system::error_code error;
		socket->receive_from(boost::asio::buffer(recv_buf),remote_endpoint, 0, error);
		if (error && error != boost::asio::error::message_size)
       		throw boost::system::system_error(error);
	}
	catch (std::exception& e)
  	{
   		std::cerr << e.what() << std::endl;
  	}
}

void UdpJpgFrameStreamer::run(){
	std::thread streamer(&UdpJpgFrameStreamer::runStream, this);
	streamer.detach();
}

void UdpJpgFrameStreamer::runStream(){
	while(1){
		
		if(ready_to_send){
			uploadFrame();
			mtx.lock();
			ready_to_send = false;
			mtx.unlock();
		}
		//bez tego nie dziala
		std::this_thread::sleep_for(std::chrono::microseconds(1));
	}
}

void UdpJpgFrameStreamer::pushFrame(Mat frame){
	mtx.lock();
	frame_to_send = frame;
	ready_to_send = true;
	mtx.unlock();
}

void UdpJpgFrameStreamer::uploadFrame(){
	std::vector<uchar> buffer;
	mtx.lock();
    imencode(".jpg", frame_to_send, buffer, compression_params);//czwarty parametr to kompresja obrazku
	mtx.unlock();
	boost::system::error_code ignored_error;
    string chunk=string(buffer.begin(),buffer.end());
	
	try{
		waitForClient();
	    socket->send_to(boost::asio::buffer(chunk),remote_endpoint, 0, ignored_error);
	}

	catch (std::exception& e)
  	{
   		std::cerr << e.what() << std::endl;
  	}
}

