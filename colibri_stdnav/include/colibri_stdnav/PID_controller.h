#ifndef _PID_CONTROLLER_H_
#define _PID_CONTROLLER_H_

#include <cmath>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>

/*
* delta form PI controller: u_delta(k) = Kp ( e(k)-e(k-1)  + Ts /Ti *e(k) + Td/Ts *(e(k) - 2e(k-1) + e(k-2)) )  ~=~ Kp*(e(k)-e(k-1))  + Ki*e(k) + Kd*(e(k) - 2e(k-1) + e(k-2))
*							= Kp(1+Ts/Ti+ Td/Ts) * e(k) - Kp(1+2Td/Ts) * e(k-1) + Kp*Td/Ts*e(k-2)
*							= a*e(k) - b*e(k-1) +c*e(k-2)	;  a = Kp(1+Ts/Ti+ Td/Ts) ,b = Kp(1+2Td/Ts), c = Kp*Td/Ts;
*							~ (Kp+Ki+Kd)*e(k) -(Kp+2Kd)*e(k-1) + Kd*e(k-2);
* ctrl output : u(k) = u(k-1)+u_delta(k)
*/

template <typename T> 	
bool Saturation( const T & bound, T * in_out) {

	if(0 == in_out)
	{
		std::cout << " in_out param in exception" <<std::endl;
		return false;
	}
		
	if(*in_out > bound) {
		*in_out = bound;
		
	} else if(*in_out < (-1.0 * bound)) {
		*in_out = -bound;
		
	} else {
	
		return false;
	}

	return true;
}

class PID_Controller
{
	public:
		struct st_PID_param
		{
			st_PID_param(double init_kp = 1., double init_ki = 0., double init_kd = 0.) : Kp(init_kp), Ki(init_ki), Kd(init_kd) {
				u_ref = 0.;
				a = 0.;
				b = 0.;
				c = 0.;
				bound4delta = 0.;
				error_1 = 0.;
				error_2 = 0.;
				u_delta = 0.;
				u_out = 0.; 	
			};
			
			double u_ref;
			double Kp;
			double Ki;
			double Kd;
			double a;
			double b;
			double c;
			double bound4delta;
			double error_1;
			double error_2;
			double u_delta;
			double u_out;
		};
		typedef st_PID_param PID_param_st;

		PID_param_st ctrl_param_;

		PID_Controller();
		PID_Controller(const double & kp, const double & ki, const double & kd, const double & bound = 4.);

		~PID_Controller();
		
		void SetControllerParam(const double & kp, const double & ki, const double & kd);

		void Regulator(const double & u_r, const double & u_fb);
	
};

#endif
