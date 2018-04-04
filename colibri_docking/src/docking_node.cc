#include "docking.h"
#include "colibri_action.h"


using namespace Eigen;
int main(int argc, char *argv[]) {
	ros::init(argc, argv, "Corner_recgnize_node");
	DockHandle dockObj;
	int delay_cnt = 0;
	sleep(1);
	ros::Rate loop_rate(10);

	float last_angular = 0.0;
	float last_linear = 0.0;
	float last_corner = 90.;
	nav_action actionObj;
	//dockObj.LoadData();
	DockCtrl dockingCtrlObj;
	DockCtrl::dock_dis cur_dock_dis = {10., 10., 10.};
	bool ready4rot = false;
	float tgt_yaw = 0.0;
	bool lock = false;
	unsigned int rot_finish = 0;

	float tmp_action_cmd_t[2] = {0.0, 0.0};
	float* ptr_action_cmd_t = tmp_action_cmd_t;
	float ang_vel = 0.785;
	float tol = 2.0;
	int cnt = 0;

	float k = 1.0;
	
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
			
					cout<<"Corner dir and dis: "<<dockObj.corner_dir_angle_<<" deg "<<dockObj.rho_filter_(0, dockObj.avg_corner_index_)<< " m" <<endl;
					cur_dock_dis.a = dockObj.rho_filter_(0, dockObj.segs_vec_[dockObj.dock_seg_index_].lower);
					cur_dock_dis.b = dockObj.rho_filter_(0, dockObj.avg_corner_index_);
					cur_dock_dis.c = dockObj.rho_filter_(0, dockObj.segs_vec_[dockObj.dock_seg_index_].upper);

				} 
				else {
					cur_dock_dis.a = dockObj.rho_filter_(0, dockObj.last_lower_);
					int mid_index = (dockObj.last_lower_ + dockObj.last_upper_) / 2.0;
					cur_dock_dis.b = dockObj.rho_filter_(0, mid_index);
					cur_dock_dis.c = dockObj.rho_filter_(0, dockObj.last_upper_);
				}
				cout<<"cur_dock_dis: a / b /c : "<<cur_dock_dis.a<<" "<<cur_dock_dis.b<<" "<<cur_dock_dis.c<<endl;

				dockingCtrlObj.vertex_diff_angle_ = CalcVertexInTriangle(cur_dock_dis.c, cur_dock_dis.b,	dockObj.dock_length_)
													 			  - CalcVertexInTriangle(cur_dock_dis.a, cur_dock_dis.b, dockObj.dock_length_);
				
				dockingCtrlObj.VelSatuarting(cur_dock_dis.b);
				if(cur_dock_dis.b  < 0.45) {
					dockingCtrlObj.linear_vel_ = 0.0;
				}

				dockingCtrlObj.delta_angle_ = 0.5 * dockObj.corner_dir_angle_ + 0.5 * last_corner - 90.;
				if(abs(dockingCtrlObj.delta_angle_) < 1.0) {
					dockingCtrlObj.delta_angle_ = 0.0;
				}
				
				if(dockObj.dock_seg_index_ == dockObj.non_dockseg_) {
					dockingCtrlObj.delta_angle_ = 0.0;
					dockingCtrlObj.linear_vel_ = 0.0;
					dockingCtrlObj.angular_vel_ = 0.0;
				}
				
				dockingCtrlObj.HeadingCtrl(cur_dock_dis, dockingCtrlObj.delta_angle_, dockingCtrlObj.vertex_diff_angle_);		
				dockObj.dock_cmd_vel_.linear.x = FirstOderFilter(0.95, dockingCtrlObj.linear_vel_, last_linear);
				dockObj.dock_cmd_vel_.angular.z = FirstOderFilter(0.95, dockingCtrlObj.angular_vel_, last_angular);

				if(cur_dock_dis.b  < 0.45 && dockingCtrlObj.delta_angle_ < 0.5 && lock == false) {

					cnt++;
					if(cnt>30) {
						ready4rot = true;
						tgt_yaw = dockObj.cartodom_yaw_ - 180.0;
						AngleConstraint(tgt_yaw);
						lock = true;
						cnt = 12;
					}
				}
				if(ready4rot) {
					ptr_action_cmd_t = actionObj.StillRotatingAction(&dockObj.cartodom_yaw_, &tgt_yaw, ang_vel,tol,&rot_finish);
					dockObj.dock_cmd_vel_.linear.x = *ptr_action_cmd_t;
					dockObj.dock_cmd_vel_.angular.z = *(ptr_action_cmd_t+1);
					if(rot_finish==1) {
						dockObj.dock_cmd_vel_.angular.z = 0.03 *(tgt_yaw - dockObj.cartodom_yaw_);
					}
					
				}
				cout<<"tgt_yaw / cur_yaw = "<<tgt_yaw<<" "<<dockObj.cartodom_yaw_<<endl;
				if(rot_finish && (dockObj.dock_seg_index_ != dockObj.non_dockseg_)) {
					ready4rot = false;
					cnt = 0;
					lock = false;
					rot_finish = 0;
				}
				
				
				dockObj.pub_twist_.publish(dockObj.dock_cmd_vel_);

				last_angular = dockingCtrlObj.angular_vel_;
				last_linear = dockingCtrlObj.linear_vel_;
				last_corner = dockObj.corner_dir_angle_;

			}
			ros::spinOnce();
			loop_rate.sleep();
		}

	
	}

	return 0;
}

