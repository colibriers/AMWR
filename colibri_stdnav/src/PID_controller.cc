#include "PID_controller.h"

bool Saturation(const double & bound, double * input) {

  if(*input > bound) {
		*input = bound;
		
	} else if(*input < (-1.0 * bound)) {
		*input = -bound;
		
	} else {
	
		return false;
	}

	return true;
}

PID_controller::PID_controller() {  
	ctrl_param_.u_ref = 0.0;

	ctrl_param_.Kp = 0.2;  
	ctrl_param_.Ki = 0.6;
	ctrl_param_.Kd = 0.0;

	ctrl_param_.a = ctrl_param_.Kp + ctrl_param_.Ki + ctrl_param_.Kd;
	ctrl_param_.b = -1.0 * (ctrl_param_.Kp + 2.0 * ctrl_param_.Kd);
	ctrl_param_.c = ctrl_param_.Kd;

	ctrl_param_.bound4delta = 0.4;

	ctrl_param_.error_1 = 0.0;
	ctrl_param_.error_2 = 0.0;

	ctrl_param_.u_delta = 0.0;
	ctrl_param_.u_out = 0.0;
}  

PID_controller::PID_controller(const double & kp, const double & ki, const double & kd, const double & bound) {  
	ctrl_param_.Kp = kp;  
	ctrl_param_.Ki = ki;
	ctrl_param_.Kd = kd;

	ctrl_param_.a = ctrl_param_.Kp + ctrl_param_.Ki + ctrl_param_.Kd;
	ctrl_param_.b = -1.0 * (ctrl_param_.Kp + 2.0 * ctrl_param_.Kd);
	ctrl_param_.c = ctrl_param_.Kd;

	ctrl_param_.bound4delta = bound;

}  

PID_controller::~PID_controller() {  

}

void PID_controller::SetControllerParam(const double & kp, const double & ki, const double & kd) {
	ctrl_param_.Kp = kp;  
	ctrl_param_.Ki = ki;
	ctrl_param_.Kd = kd;

	ctrl_param_.a = ctrl_param_.Kp + ctrl_param_.Ki + ctrl_param_.Kd;
	ctrl_param_.b = -1.0 * (ctrl_param_.Kp + 2.0 * ctrl_param_.Kd);
	ctrl_param_.c = ctrl_param_.Kd;

}

void PID_controller::Regulator(const double & u_r, const double & u_fb) {  
	double error = 0.0;

	ctrl_param_.u_ref = u_r;
	error = ctrl_param_.u_ref - u_fb;

	ctrl_param_.u_delta= ctrl_param_.a * error + ctrl_param_.b * ctrl_param_.error_1 + ctrl_param_.c * ctrl_param_.error_2;

	ctrl_param_.error_2 = ctrl_param_.error_1;
	ctrl_param_.error_1 = error;

	Saturation(ctrl_param_.bound4delta , &(ctrl_param_.u_delta));

	//ctrl_param_.u_out += ctrl_param_.u_delta;  	//positon mode
	ctrl_param_.u_out = ctrl_param_.u_delta;  		//delta mode 

}

