#include "PID_controller.h"

int main(void) {
	PID_controller reg1Obj;
	PID_controller reg2Obj(3. ,6., 9., 8.);
	
	reg1Obj.SetControllerParam(1.0, 2.0, 3.0);
	reg1Obj.Regulator(1.0, 6.6);

	double ref = 10;
	double fb = 8.1;
	reg2Obj.Regulator(ref, fb);

	return 0;
}
