#include "bcm2835.h"
#include "tmp102.h"
#include "l6470constants.h"
#include "motors.h"
#include "VL53L1X.h"
#include "LSM6DS3.h"
#include "filter.h"
#include <csignal>
#include <math.h>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <ctime>
#include <chrono>
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include "UdpJpgFrameStreamer.h"
#include "ContourFinding.h"
#include "CenterFinding.h"
#include "Pid.h"
#include "DataSaver.h"
using namespace cv;
using namespace std;

using boost::asio::ip::udp;

#define GPIO_TOF_1 	RPI_V2_GPIO_P1_12
#define GPIO_TOF_2 	RPI_V2_GPIO_P1_16
#define GPIO_TOF_3 	RPI_V2_GPIO_P1_18
#define GPIO_TOF_4 	RPI_V2_GPIO_P1_29
#define GPIO_TOF_5	RPI_V2_GPIO_P1_32
#define GPIO_TOF_6 	RPI_V2_GPIO_P1_31
#define GPIO_TOF_7 	RPI_V2_GPIO_P1_33
#define GPIO_TOF_8 	RPI_V2_GPIO_P1_35
#define GPIO_TOF_9 	RPI_V2_GPIO_P1_36
#define GPIO_TOF_10 RPI_V2_GPIO_P1_37
#define NDEBUG

void stepperTest();
void tofTest();
void IMUtest();
void TMPtest();
void batteryTest();
Motors *globalBoard;
VL53L1X *globalSensors[10];
uint16_t measurement[10];

void sigintHandler(int signum) {
	if (signum == SIGINT) {
		//globalBoard->Dump();
		//file.close();
		if (globalBoard != nullptr) globalBoard->stop();
		for(int i = 0; i < 10; i++)
			if (globalSensors[i] != nullptr) globalSensors[i]->disable();
		//globalBoard->stop();
		std::cout<<"TASK TERMINATED"<<endl;
		exit(signum);
	}
}


int main()
{
	//setup motors
	signal(SIGINT, sigintHandler);
	if (bcm2835_init() == 0) {
			fprintf(stderr, "Not able to init the bmc2835 library\n");
			return -1;
	}

	//IMUtest();
	//delay(2000);
	//TMPtest();
	//batteryTest();
	//stepperTest();
	//-----------------------------------------------------
	int counter = 0;
	double total_center_time = 0;
	double total_drawing_time = 0;
	int max_counter = 1000;
	Motors board( BCM2835_SPI_CS0, GPIO_RESET_OUT);
	globalBoard = &board;
	board.setUp();
	board.resetPosition();
	//-----------------------------------------------------

	//setup camera
	VideoCapture clipCapture(0);
	UdpJpgFrameStreamer streamer(2024, 64000, 80);
	ContourFinding contourFinder(1.0/6.0, 5.0/6.0);
	CenterFinding centerFinder(6);
	int duration;
	int regulation_period = 60000;
	int threshold = 50;
	double scale = 0.5;
	Pid pid(0.3, 50, 0.05, regulation_period, 40, -70, 60);
	int frame_width = clipCapture.get(3)*scale; 
  	int frame_height = clipCapture.get(4)*scale; 
	DataSaver dataSaver("test", "test", "/", frame_width, frame_height, regulation_period);

	Mat src;
	std::cout<<"Contour or center finding? (1/2)";
	int method;
	std::cin>>method;

	//sprawdzenie czy wczytano poprawnie
	if (!clipCapture.isOpened())
	{
	  	cout  << "Could not open reference to clip" << endl;
		exit(0);
	}
	streamer.run();
	//-----------------------------------------------------

	//main loop
	while(1){
		auto start1 = chrono::steady_clock::now();
		auto start = chrono::steady_clock::now();
		clipCapture.read(src);
		int center;
		pair<int, int> p; 
		std::vector<cv::Point> centers;

		if (src.empty() || src.cols == -1 || src.rows == -1)
		{
		    	printf("No image data from clip\n");
		    	break;
		}

		else
		{	
			if(method == 1)
			{
				contourFinder.setFrame(src);
				contourFinder.setScaleFactor(scale);//default is 0.5
				contourFinder.setThreshold(threshold);
				centers = contourFinder.findLineCenters();
				Mat frame = contourFinder.drawPoints(centers);
				streamer.pushFrame(frame);

				//pid

				center = round(contourFinder.getSourceFrame().cols/2);
				pid.setSetPoint(center);
				if(centers.empty()) p = pair<int, int>(0, 0);
				else p = pid.calculateControl(centers[0].x);

				//std::cout<<"Speed: "<< -p.first << ", " << -p.second <<endl;
				//std::cout<<"Error: "<<center - centers[0].x<<std::endl;
				board.setSpeed(-p.first, -p.second);
				auto end = chrono::steady_clock::now();
				duration = chrono::duration_cast<chrono::microseconds>(end - start).count();
				//std::cout<<"Time: "<<duration<<std::endl;
			}

			else
			{
				centerFinder.setFrame(src);
				centerFinder.setScaleFactor(0.3);//default is 0.5
				std::vector<cv::Point> centers = centerFinder.findLineCenters();
				Mat frame  = centerFinder.drawPoints(centers);
				streamer.pushFrame(frame);
			}
     	}
		if(duration < regulation_period) std::this_thread::sleep_for(std::chrono::microseconds(regulation_period - duration));
		else std::cout<<"EXCEEDED REGULATION LOOP TIME BY: "<< duration - regulation_period <<endl;
		auto end1 = chrono::steady_clock::now();
		std::cout<<"Loop time: "<< chrono::duration_cast<chrono::microseconds>(end1 - start1).count()<<endl;
		if(!centers.empty()) dataSaver.setDataToTxt(-p.first, -p.second, center, center - centers[0].x, duration);
		else dataSaver.setDataToTxt(-p.first, -p.second, center, 1000000, duration);
		//clipCapture.release();
		//break;
	}

  return 0;
}

void batteryTest(){
        long positionLeft,positionRight;
        Motors board( BCM2835_SPI_CS0, GPIO_RESET_OUT);
        globalBoard = &board;
        board.setUp();
        board.resetPosition();
	auto timer_old = std::chrono::high_resolution_clock::now();
	for (int j=0; j<1000; j++){
		std::cout << board.getBatteryVoltage() << std::endl;
	}
	auto timer_value = std::chrono::high_resolution_clock::now();
	auto time_span = std::chrono::duration_cast<std::chrono::duration<double>>(timer_value - timer_old);

	std::cout << "1xBatteryV average time:" << std::endl;
	std::cout << time_span.count()/1000.0 << std::endl;
        board.stop();
}

void stepperTest(){
	long positionLeft,positionRight;
	Motors board( BCM2835_SPI_CS0, GPIO_RESET_OUT);
	globalBoard = &board;
	board.setUp();
	board.resetPosition();
	positionLeft = board.getPositionLeft();
	positionRight = board.getPositionRight();
	printf("Absolute position: Left:%lu		Right:%lu \n",positionLeft, positionRight);
	board.setSpeed(-20,-20);
	bcm2835_delay(500);
	positionLeft = board.getPositionLeft();
	positionRight = board.getPositionRight();
	printf("Absolute position: Left:%lu		Right:%lu \n",positionLeft, positionRight);
	//file <<"end_left:"<< positionLeft <<"end_right:"<< positionRight<<std::endl;

	board.stop();

	//file.close();
}

void tofTest(){

	// Log file
	//file.open("distance_log_report70cm");

	bcm2835_gpio_fsel(GPIO_TOF_1, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_2, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_3, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_4, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_5, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_6, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_7, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_8, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_9, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_10, BCM2835_GPIO_FSEL_OUTP);

	//disable all sensors first
	bcm2835_gpio_clr(GPIO_TOF_1); // górny
	bcm2835_gpio_clr(GPIO_TOF_2); // górny
	bcm2835_gpio_clr(GPIO_TOF_3);
	bcm2835_gpio_clr(GPIO_TOF_4);
	bcm2835_gpio_clr(GPIO_TOF_5);
	bcm2835_gpio_clr(GPIO_TOF_6);
	bcm2835_gpio_clr(GPIO_TOF_7);
	bcm2835_gpio_clr(GPIO_TOF_8);
	bcm2835_gpio_clr(GPIO_TOF_9);
	bcm2835_gpio_clr(GPIO_TOF_10);

	bcm2835_i2c_begin(); //begin I2C
	bcm2835_i2c_set_baudrate(40000);

	//enable sensor one and change address
	bcm2835_gpio_set(GPIO_TOF_1);
	delay(10);
	globalSensors[0] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[0]->setAddress(0x30);
	delay(10);
	puts("Sensor one started at: 0x30");

	bcm2835_gpio_set(GPIO_TOF_2);
	delay(10);
	globalSensors[1] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[1]->setAddress(0x31);
	delay(10);
	puts("Sensor two started at: 0x31");

	bcm2835_gpio_set(GPIO_TOF_3);
	delay(10);
	globalSensors[2] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[2]->setAddress(0x32);
	delay(10);
	puts("Sensor three started at: 0x32");

	bcm2835_gpio_set(GPIO_TOF_4);
	delay(10);
	globalSensors[3] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[3]->setAddress(0x33);
	delay(10);
	puts("Sensor four started at: 0x33");

	bcm2835_gpio_set(GPIO_TOF_5);
	delay(10);
	globalSensors[4] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[4]->setAddress(0x34);
	delay(10);
	puts("Sensor five started at: 0x34");

	bcm2835_gpio_set(GPIO_TOF_6);
	delay(10);
	globalSensors[5] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[5]->setAddress(0x35);
	delay(10);
	puts("Sensor six started at: 0x35");

	bcm2835_gpio_set(GPIO_TOF_7);
	delay(10);
	globalSensors[6] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[6]->setAddress(0x36);
	delay(10);
	puts("Sensor seven started at: 0x36");

	bcm2835_gpio_set(GPIO_TOF_8);
	delay(10);
	globalSensors[7] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[7]->setAddress(0x37);
	delay(10);
	puts("Sensor eight started at: 0x37");

	bcm2835_gpio_set(GPIO_TOF_9);
	delay(10);
	globalSensors[8] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[8]->setAddress(0x38);
	delay(10);
	puts("Sensor nine started at: 0x38");

	bcm2835_gpio_set(GPIO_TOF_10);
	delay(10);
	globalSensors[9] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[9]->setAddress(0x39);
	delay(10);
	puts("Sensor ten started at: 0x39");


	for(int i=0; i<10; i++){
		globalSensors[i]->startContinuous(20);
		delay(10);
	}
	printf("started coninous");

	for(int j=0; j<1000; j++){
	//while(1){
		//for(int i=0; i<10; i++){
		//			measurement[i] = globalSensors[i]->readData(1);
		//			printf("%d:%5d ",i,measurement[i]);
		//}
				measurement[5] = globalSensors[5]->readData(1);
				printf("5:%5d ",measurement[5]);

		printf("\n");
		delay(20);
	}

	for(int i=0; i<10; i++){
		globalSensors[i]->disable();
		delay(10);
	}

	//file.close();
	//board.stop();
}

void IMUtest(){
	LSM6DS3 SensorOne( I2C_MODE, 0x6B);

	if( SensorOne.begin() != 0 )
	{
		  printf("Problem starting the sensor \n");
		  return;
	}
	else
	{
		  printf("Sensor with CS1 started, awaiting calibration.\n");
		  for (int i = 10; i>0; i--){
			  printf("%d\n",i);
			  delay(1000);
		  }
	}
	int i, n = 1000;

	// Acceleration
	float  ax, ay, az;
	ax = SensorOne.readFloatAccelX();
	ay = SensorOne.readFloatAccelY();
	az = SensorOne.readFloatAccelZ();

	// Gyro
	float rgx, rgy, rgz;
	const float offX = 3.755909, offY = -5.435584, offZ = -3.461446;
	const float offXRad = -0.000329, offYRad = -0.000224, offZRad = 0.000880;
	rgx = (SensorOne.readFloatGyroX() - offX)*M_PI/180 - offXRad;
	rgy = (SensorOne.readFloatGyroY() - offY)*M_PI/180 - offYRad;
	rgz = (SensorOne.readFloatGyroZ() - offZ)*M_PI/180 - offZRad;

	// Sum of gyro readings
	float sumgx = 0, sumgy = 0, sumgz = 0;

	// Tilt
	float ty = atan2(ax,sqrt(ay*ay+az*az));

	// Filtering
	float param = 0.1;
	float freq = 100;
	filter f(ty,param,freq);
	float f_gy, f_ty;

	// Log file
	char filename [25];
	sprintf(filename,"imu_log_report_%.3f_%04.0f", param, freq);
	//file.open(filename);

	for(i=0; i<n; i++){
		//Get all parameters
		printf("\nAccelerometer:\n");
		ax = SensorOne.readFloatAccelX();
		ay = SensorOne.readFloatAccelY();
		az = SensorOne.readFloatAccelZ();
		printf(" X = %f\n",ax);
		printf(" Y = %f\n",ay);
		printf(" Z = %f\n",az);
		printf(" ACC = %f\n", sqrt(ax*ax+ay*ay+az*az));

		printf("\nGyroscope:\n");
		rgx = (SensorOne.readFloatGyroX() - offX)*M_PI/180 - offXRad;
		rgy = (SensorOne.readFloatGyroY() - offY)*M_PI/180 - offYRad;
		rgz = (SensorOne.readFloatGyroZ() - offZ)*M_PI/180 - offZRad;
		sumgx += rgx;
		sumgy += rgy;
		sumgz += rgz;
		printf(" X = %f\n",rgx);
		printf(" Y = %f",rgy);
		printf(" Z = %f\n",rgz);

		printf("\nTilt:\n");
		ty = atan2(ax,sqrt(ay*ay+az*az));
		printf(" Tilt = %f", ty);

		printf("\nAfter filtering:\n");
		f_ty = f.getAngle(ty,rgy);
		f_gy = f.getGyro();
		printf(" Gyro Y: %f", f_gy);
		printf(" Tilt Y: %f\n", f_ty);

		printf("\nThermometer:\n");
		printf(" Degrees C = %f\n",SensorOne.readTempC());

		printf("\nSensorOne Bus Errors Reported:\n");
		printf(" All '1's = %d\n",SensorOne.allOnesCounter);
		printf(" Non-success = %d\n",SensorOne.nonSuccessCounter);

		//Write to file
		//file << i << " " << rgy << " " << ty << " " << f_gy << " " << f_ty << std::endl;
		delay(1000/freq);
	}
	//file.close();

	printf(" Average Gyro X = %f\n", sumgx/n);
	printf(" Average Gyro Y = %f\n", sumgy/n);
	printf(" Average Gyro Z = %f\n", sumgz/n);

	SensorOne.close_i2c();
}

void TMPtest(){ 
	tmp102 czujnik(0x48,"/dev/i2c-1");
	printf("Rys temperature: %f \n",czujnik.readTemperature());

}
