#include "docking.h"

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "Docking_test_node");
	ScanHandle scanHandObj;

	ros::Rate loop_rate(10);
	int delay_cnt = 0;
	
	while(ros::ok()) {
		if(delay_cnt < DELAY_CNT_MAX){
			delay_cnt++;
			ros::spinOnce();
			loop_rate.sleep();
		}
		Eigen::Matrix<float, 2, 3> tmp_diff;
		tmp_diff << 1., 2., 3., 4., 5., 6.;
		float diff_norm = tmp_diff.col(1).norm();

		if(delay_cnt >= DELAY_CNT_MAX){
			delay_cnt = DELAY_CNT_MAX;
		
			ros::spinOnce();
			loop_rate.sleep();
		}


	}

	return 0;
}

