#include "PID_controller.h"

PID_controller::PID_controller()
{  
    PID_param.u_ref = 0.0;
	
    PID_param.Kp = 0.2;  
    PID_param.Ki = 0.6;
	PID_param.Kd = 0.0;
	
	PID_param.a = PID_param.Kp + PID_param.Ki + PID_param.Kd;
	PID_param.b = -1.0 * (PID_param.Kp + 2.0 * PID_param.Kd);
	PID_param.c = PID_param.Kd;
	
    PID_param.bound4delta = 0.4;
	
    PID_param.error_1 = 0.0;
	PID_param.error_2 = 0.0;
	
    PID_param.u_delta = 0.0;
	PID_param.u_out = 0.0;
}  

PID_controller::~PID_controller()
{  

}

void PID_controller::InitRegulatorParam(float kp, float ki, float kd, float bound)
{
    PID_param.Kp = kp;  
    PID_param.Ki = ki;
	PID_param.Kd = kd;
	
	PID_param.a = PID_param.Kp + PID_param.Ki + PID_param.Kd;
	PID_param.b = -1.0 * (PID_param.Kp + 2.0 * PID_param.Kd);
	PID_param.c = PID_param.Kd;
	
	PID_param.bound4delta= bound;
}

void PID_controller::Regulator(float u_r, float u_fb, float* u_out)  
{  
    float error = 0.0;
	
    PID_param.u_ref = u_r;
    error = PID_param.u_ref - u_fb;
   
    PID_param.u_delta= PID_param.a * error + PID_param.b * PID_param.error_1 + PID_param.c * PID_param.error_2;

	PID_param.error_2 = PID_param.error_1;
	PID_param.error_1 = error;
	
	DeltaOutBound(&(PID_param.u_delta), PID_param.bound4delta);
	
	//PID_param.u_out += PID_param.u_delta;  	//positon mode
    PID_param.u_out = PID_param.u_delta;  		//delta mode 
  
    *u_out = PID_param.u_out;  
}

void PID_controller::DeltaOutBound(float* input, float bound)
{
    if(*input > bound)
   	{
		*input = bound;
	}
	else if(*input < (-1.0 * bound))
	{
		*input = -bound;
	}
	else
	{

	}
}



