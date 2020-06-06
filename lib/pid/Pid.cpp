#include "Pid.h"

Pid::Pid(double k, double ti, double td, double t, int u_work_point, int u_min, int u_max) : 
k_(k), ti_(ti), td_(td), t_(t/1000000), u_work_point_(u_work_point), u_min_(u_min), u_max_(u_max), set_point_(0), ui_past_(0), e_past_(0) {};

void Pid::setSetPoint(int set_point){
	set_point_ = set_point;
}

std::pair<int, int> Pid::calculateControl(int process_output){
	int e = set_point_ - process_output;
	double up = k_*e;
	double ui = ui_past_ + (k_ / ti_) * t_ * (e_past_ + e) / 2;
	double ud = k_ * td_ * (e - e_past_) / t_;
	int u = round(up + ui + ud);
	e_past_ = e;
	ui_past_ = ui;
	if(u > u_max_) u = u_max_;
	if(u < u_min_) u = u_min_;
	return std::make_pair(u_work_point_ - u, u_work_point_ + u);
}
