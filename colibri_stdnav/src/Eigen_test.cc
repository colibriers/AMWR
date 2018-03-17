#include <eigen3/Eigen/Core>
#include <iostream>
#include <string>
#include <algorithm>

#define NUM_RAY4CA 181
int main(void) {
	Eigen::Array<float, 1 , 5> test_var;
	test_var(0) = 9.9;
	test_var(1) = 3.3;
	test_var(2) = 9.9;
	test_var(3) = 3.3;	
	test_var(4) = 100.;	
	std::cout << test_var(2) <<std::endl;
	Eigen::Array<float, 1 , 10> test_var2;
	Eigen::Array<float, 1 , 10> test_var3(2 * test_var2.setOnes());

	test_var2 = 57.295 * (1.0 * test_var3.inverse()).asin();
	Eigen::Array<int, 1 , 5> tmp = test_var.cast<int>();
	test_var3.setLinSpaced(10, 1, 12);

	Eigen::Array<int, 1, 181> index_angle;
	index_angle.setLinSpaced(NUM_RAY4CA, 0, 180);


	Eigen::Array<int, 1, NUM_RAY4CA> tmpOnes;
	tmpOnes.setOnes();
	Eigen::Array<int, 1, NUM_RAY4CA> range_num = tmpOnes * 10;

	Eigen::Array<int, 1, NUM_RAY4CA> tmpcom;
	tmpcom = index_angle.min(range_num);

	Eigen::Array<float, 1, NUM_RAY4CA> tmpcomA;
	tmpcomA = 0.1 * tmpcomA.setOnes();
	Eigen::Array<float, 1, NUM_RAY4CA> tmpcomB;
	tmpcomB = 9.9 * tmpcomB.setOnes();

	Eigen::Array<float, 1, NUM_RAY4CA> tmpcomC;
	tmpcomC = (index_angle > range_num).select(tmpcomA, tmpcomB);
	tmpcomC = tmpcomC - .1;
	tmpcomC = tmpcomC.inverse();
	Eigen::Array<float, 1, NUM_RAY4CA> tmpcomD, tmpE;
	tmpcomD = tmpcomC.inverse();

	tmpE = -tmpcomB;
	int i = 0;
	int j = 0;
	float  seq_max_val = test_var.minCoeff(&i); 
	Eigen::Array<float, 1 , 5> test_divid;
	test_divid << 2.0, 3.0, 2.0, 5, 10.;
	test_divid = test_var / test_divid;
	std::cout << test_var.tail(3) <<std::endl;
	std::cout << test_var.reverse() <<std::endl;

	float arr[5];
	std::fill(arr,arr+5, 1.1);
	//memset(arr, 20.0, sizeof(arr));
	Eigen::Map< Eigen::Array<float, 1 , 5> >(arr, test_divid.rows(),test_divid.cols()) = test_divid;

	
	return 0;
}
