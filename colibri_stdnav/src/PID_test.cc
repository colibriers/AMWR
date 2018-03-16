#include "PID_controller.h"

int main(void) {
	PID_Controller reg1Obj;
	PID_Controller reg2Obj(5., 6., 7.);
	PID_Controller reg3Obj(1., 2., 3., 10.);
		
	reg1Obj.SetControllerParam(1.0, 2.0, 3.0);
	reg1Obj.Regulator(1.0, 0.8);

	double ref = 10;
	double fb = 8.1;
	reg2Obj.Regulator(ref, fb);

	int bnd = 18;
	int result = 17;
	int * ptr_result = &result;
	bool sat = Saturation<int>(bnd, ptr_result);

	std::cout << *ptr_result <<" " << result <<std::endl;

	return 0;
}
