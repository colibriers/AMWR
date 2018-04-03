#include "docking.h"

using namespace Eigen;
int main(int argc, char *argv[]) {
	ros::init(argc, argv, "Corner_recgnize_node");
	DockHandle dockObj;
	int delay_cnt = 0;
	sleep(1);
	ros::Rate loop_rate(10);

	//dockObj.LoadData();
	DockCtrl dockingCtrlObj;
	DockCtrl::dock_dis cur_dock_dis = {10., 10., 10.};
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
				if(dockObj.dock_seg_index_ != dockObj.non_dockseg_) {
					dockObj.CalcMaxDis2Segs(dockObj.scan_filter_, dockObj.dock_seg_index_);
					dockObj.CalcMatchCornerIndex(dockObj.scan_filter_, dockObj.dock_seg_index_);
					dockObj.CalcAvgCornerDir();
				}
				cout<<"Corner dir: "<<dockObj.corner_dir_angle_<<endl;
				
				cur_dock_dis.a = dockObj.rho_filter_(0, dockObj.segs_vec_[dockObj.dock_seg_index_].lower);
				cur_dock_dis.b = dockObj.rho_filter_(0, dockObj.avg_corner_index_);
				cur_dock_dis.c = dockObj.rho_filter_(0, dockObj.segs_vec_[dockObj.dock_seg_index_].upper);
				dockingCtrlObj.VelSatuarting(cur_dock_dis.b);
				float delta_angle = dockObj.corner_dir_angle_ - 90.;
				dockingCtrlObj.HeadingCtrl(cur_dock_dis, delta_angle);
				dockObj.dock_cmd_vel_.linear.x = dockingCtrlObj.linear_vel_;
				dockObj.dock_cmd_vel_.angular.z = dockingCtrlObj.angular_vel_;
				dockObj.pub_twist_.publish(dockObj.dock_cmd_vel_);


			}
			ros::spinOnce();
			loop_rate.sleep();
		}

	
	}

	return 0;
}

