#include "docking.h"

using namespace Eigen;
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
				scanHandObj.Polar2Cartesian();
				scanHandObj.CalcBreakerMarker(scanHandObj.rho_filter_, scanHandObj.scan_filter_);
				scanHandObj.CalcContiSegs();
				scanHandObj.dock_seg_index_ = 8;
				scanHandObj.CalcMaxDis2Segs(scanHandObj.scan_filter_, scanHandObj.dock_seg_index_); 
				scanHandObj.CalcMatchCornerIndex(scanHandObj.scan_filter_);

				
			}
			ros::spinOnce();
			loop_rate.sleep();
		}

	
	}

	return 0;
}

