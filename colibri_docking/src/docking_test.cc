#include "docking.h"

using namespace Eigen;
int main(int argc, char *argv[]) {
	ros::init(argc, argv, "Corner_recgnize_node");
	DockHandle dockObj;
	int delay_cnt = 0;
	sleep(1);
	ros::Rate loop_rate(10);

	//dockObj.LoadData();
	while(ros::ok()) {
		if(delay_cnt < DELAY_CNT_MAX){
			delay_cnt++;
			ros::spinOnce();
			loop_rate.sleep();
		}
		
		if(delay_cnt >= DELAY_CNT_MAX){
			delay_cnt = DELAY_CNT_MAX;
			if(dockObj.refresh_flag_ == true) {
				dockObj.refresh_flag_ = false;
				dockObj.MedFilter();
				dockObj.Polar2Cartesian();
				dockObj.CalcBreakerMarker(dockObj.rho_filter_, dockObj.scan_filter_);
				dockObj.CalcContiSegs();
				dockObj.CalcDockSegIndex();
				if(dockObj.dock_seg_index_ != 255) {
					dockObj.CalcMaxDis2Segs(dockObj.scan_filter_, dockObj.dock_seg_index_);
					dockObj.CalcMatchCornerIndex(dockObj.scan_filter_, dockObj.dock_seg_index_);
					dockObj.CalcAvgCornerDir();
				}

				cout<<"Corner dir: "<<dockObj.corner_dir_angle_<<endl;
			}
			ros::spinOnce();
			loop_rate.sleep();
		}

	
	}

	return 0;
}

