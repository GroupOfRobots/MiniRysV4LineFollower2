#include <iostream>
#include <string>
#include <stdio.h>
#include <cmath>
using namespace std;

#ifndef PID_H
#define PID_H

class Pid {
	private:
		int u_max_;
		int u_min_;
		int set_point_;
		int u_work_point_;
		double k_;
		double ti_;
		double td_;
		double t_;
		int ui_past_;
		int e_past_;
		
	public:
		Pid(double k, double ti, double td, double t, int u_work_point, int u_min, int u_max);
		void setSetPoint(int set_point);
		std::pair<int, int> calculateControl(int process_output);
};

#endif
