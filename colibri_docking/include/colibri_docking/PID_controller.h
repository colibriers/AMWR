#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <ctime>

#ifndef _PID_CONTROLLER_H_
#define _PID_CONTROLLER_H_

/*
* delta form PI controller: u_delta(k) = Kp ( e(k)-e(k-1)  + Ts /Ti *e(k) + Td/Ts *(e(k) - 2e(k-1) + e(k-2)) )  ~=~ Kp*(e(k)-e(k-1))  + Ki*e(k) + Kd*(e(k) - 2e(k-1) + e(k-2))
*							= Kp(1+Ts/Ti+ Td/Ts) * e(k) - Kp(1+2Td/Ts) * e(k-1) + Kp*Td/Ts*e(k-2)
*							= a*e(k) - b*e(k-1) +c*e(k-2)	;  a = Kp(1+Ts/Ti+ Td/Ts) ,b = Kp(1+2Td/Ts), c = Kp*Td/Ts;
*							~ (Kp+Ki+Kd)*e(k) -(Kp+2Kd)*e(k-1) + Kd*e(k-2);
* ctrl output : u(k) = u(k-1)+u_delta(k)
*/

typedef struct st_PID_param
{
	float u_ref;
	float Kp;
	float Ki;
	float Kd;
	float a;
	float b;
	float c;
	float bound4delta;
	float error_1;
	float error_2;
	float u_delta;
	float u_out;
}PID_param_struct;

class PID_controller
{
	public:
		
		PID_param_struct PID_param;

		PID_controller();
		~PID_controller();
		
		void InitRegulatorParam(float kp, float ki, float kd, float bound);
		void Regulator(float u_r, float u_fb, float* u_out);

	private:
		
		void DeltaOutBound(float *input, float bound);
	
};


#endif
