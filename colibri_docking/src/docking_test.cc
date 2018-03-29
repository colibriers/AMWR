#include "docking.h"

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "Docking_test_node");
	ScanHandle scanHandObj;
	int delay_cnt = 0;
	sleep(1);
	ros::Rate loop_rate(10);

	scanHandObj.LoadData();
	while(ros::ok()) {
		if(delay_cnt < DELAY_CNT_MAX){
			delay_cnt++;
			ros::spinOnce();
			loop_rate.sleep();
		}
		
		if(delay_cnt >= DELAY_CNT_MAX){
			delay_cnt = DELAY_CNT_MAX;
			if(scanHandObj.refresh_flag_ == false) {
				scanHandObj.refresh_flag_ = false;
				scanHandObj.MedFilter();
				//scanHandObj.Polar2Cartesian(); 
				scanHandObj.Polar2Cartesian(scanHandObj.rho_origin_, scanHandObj.scan_orgin_);
				scanHandObj.CalcBreakerMarker(scanHandObj.rho_origin_, scanHandObj.scan_orgin_);
				scanHandObj.CalcContiSegs();
				//scanHandObj.CalcMaxDis2Segs(const Eigen::Matrix<float, 2, Eigen::Dynamic> & scan_xy); 
				//scanHandObj.CalcMatchCornerIndex(const Eigen::Matrix<float, 2, SCAN_RAY_NUM> & scan_xy);
				//scanHandObj.CalcCornerFunc(const Eigen::Matrix<float, 2, SCAN_RAY_NUM> & scan_xy, const int &index, const int &width);

				
			}
			ros::spinOnce();
			loop_rate.sleep();
		}

	
	}

	return 0;
}

