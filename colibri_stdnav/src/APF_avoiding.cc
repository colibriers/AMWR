#include "APF_avoiding.h"

APF::APF(float init_goal_dir): goal_dir_(init_goal_dir),
																 max_passfcn_val_(0.), min_laser_(20.),
																 min_laser_dir_(90.), fwd_maxpass_cnt_(0),
																 bwd_maxpass_cnt_(0), ctrl_v_(0.), ctrl_vth_(0.),
																 angle_adj_(0.), apf_alarm_(0), refresh_flag_(false){
				  
	std::fill(abstract_pf_, abstract_pf_ + NUM_RAY4CA, 20.);
	
	scan_sub4ca_ = nh_ca_.subscribe<sensor_msgs::LaserScan>("/scan", 1, &APF::ScanCallBack, this);
	env_sub4safe_ = nh_ca_.subscribe<colibri_msgs::EnvSecurity>("/env_secure", 1, &APF::EnvSecurityCallBack,this);
	pf_Pub4dbg_ = nh_ca_.advertise<colibri_msgs::AngPotnEngy>("/pf_dbg", 5); 
}

APF::~APF()
{
	
}

void APF::CalcPhiRange(const Eigen::Array<int, 1, NUM_RAY4CA> &range_num) {
	Eigen::Array<int, 1, NUM_RAY4CA> index_angle;
	index_angle.setLinSpaced(NUM_RAY4CA, 0, 180);
	Eigen::Array<int, 1, NUM_RAY4CA> bnd;
	
	bnd.setOnes();
	Eigen::Array<int, 1, NUM_RAY4CA> index_diff = index_angle - range_num;
	phi_start_vec_ = index_diff.max(bnd);

	bnd = NUM_RAY4CA * bnd;
  index_diff = index_angle + range_num;
	phi_end_vec_ = index_diff.min(bnd);
}

void APF::CalcKpPhi(const float & vel) {
  CalcDsrVc(vel);
	Array_CA kp_phi_tmp = (scan4ca_ - dsr_v_).inverse();

	Array_CA val;
	val = KP_PHI_INF * val.setOnes();

	Array_CA bnd;
	bnd = dsr_v_ * bnd.setOnes();
  kp_phi_vec_ = (bnd >= scan4ca_).select(val, kp_phi_tmp);
	
	bnd = D_M * bnd.setOnes();
	val = 1 / (D_M - dsr_v_) * val.setOnes();
	kp_phi_vec_ = (scan4ca_ >= bnd).select(val, kp_phi_tmp);
}

void APF::CalcPhiParam(const float & velocity) {

  Eigen::Array<int, 1, NUM_RAY4CA> range_num;
	
	delta_phi_vec_ = RAD2DEG * (D_SF * scan4ca_.inverse()).asin();
	range_num = (delta_phi_vec_ / RAY_RESOL4CA).cast<int>();
	CalcPhiRange(range_num);	

	CalcKpPhi(velocity);

	krf_vec_ = kp_phi_vec_;
}

void APF::CalcKafTheta(const float& goal_inlaser) {

	Eigen::Array<float, 1, NUM_RAY4CA> index_angle, tmp_delta;
	index_angle.setLinSpaced(NUM_RAY4CA, 0., 180.);

	tmp_delta = index_angle - goal_inlaser; 		// unit in degree
	LimitAngle(tmp_delta);
	kaf_vec_ = (tmp_delta * DEG2RAD).cos();	//dir_goal_in_laser is 0~180 degree from right side
}

void APF::CalcKrfTheta(void) {
	for(int theta_index = 0; theta_index < NUM_RAY4CA; theta_index++) {
		for(int phi_index = 0; phi_index < NUM_RAY4CA; phi_index++) {
			if((theta_index >= phi_start_vec_(phi_index)) && (theta_index <= phi_end_vec_(phi_index))) {
				if(krf_vec_(theta_index) < kp_phi_vec_(phi_index)) {	
					krf_vec_(theta_index) = kp_phi_vec_(phi_index);	// to obtain the max [Krf(phi,theta)] as Krf(theta)				
				}
			}		
		}
	}
}

void APF::CalcCorrectedKrf(const float & krf_corrector) {
	Eigen::Array<float, 1, NUM_RAY4CA> index_angle, tmp_delta;
	index_angle.setLinSpaced(NUM_RAY4CA, 0., 180.);
	
	Eigen::Array<float, 1, NUM_RAY4CA> puv = (index_angle - 90.0) / 90.;

	krf_vec_ = (1.0 + krf_corrector * puv.pow(2)) * krf_vec_;
}

void APF::CalcPassFcn(const int  & seq_flag, const bool & concern_apf) {
	int col = 90;
	Array_CA tmp_vec;
	if(concern_apf) {
		passfcn_vec_ = kaf_vec_ / krf_vec_;
		
	} else {
		passfcn_vec_ = 1. / kaf_vec_;
		
	}

	if(1 == seq_flag) {
		max_passfcn_val_ = passfcn_vec_.maxCoeff(&col);
		maxfcn_fwdbnd_ = col;
		maxfcn_bwdbnd_ = maxfcn_fwdbnd_;
		
	} else if(-1 == seq_flag) {
		tmp_vec = passfcn_vec_.reverse();
		max_passfcn_val_ = tmp_vec.maxCoeff(&col);
		maxfcn_bwdbnd_ = ANGLE4CA_END - col;
		maxfcn_fwdbnd_ = maxfcn_bwdbnd_;
		
	} else {
		max_passfcn_val_ = passfcn_vec_.maxCoeff(&col);
		maxfcn_fwdbnd_ = col;
		
		tmp_vec = passfcn_vec_.reverse();
		max_passfcn_val_ = tmp_vec.maxCoeff(&col);
		maxfcn_bwdbnd_ = ANGLE4CA_END - col;
	}	
}

void APF::CalcPassFcnWithoutRPF(void) {	
	int col = 90;
	passfcn_vec_ = D_M * kaf_vec_;
	
	//0 to 180 deg search max passfcn val  and  max passfcn's bound index  locate at right side
	max_passfcn_val_ = passfcn_vec_.maxCoeff(&col); 
	angle_adj_ = col - 90.0; 
}

void APF::CalcAdjDir(void) {
	int mid_num = 90;

	if(abs(maxfcn_fwdbnd_ - maxfcn_bwdbnd_) < 3)
	{
		angle_adj_ =  (maxfcn_fwdbnd_ + maxfcn_bwdbnd_) / 2.0 - mid_num;
		return;
	}

	float delta = MIN(PASSVAL_TOLLERENCE * max_passfcn_val_, MAX_PASSFCN_SCOPE);

	//backward directrion(CW 180->0) search record
	for(int m = 0; m <= mid_num; m++) {
		if(abs(passfcn_vec_(maxfcn_bwdbnd_ - m) - max_passfcn_val_) < delta) {
			bwd_maxpass_cnt_++;
		} else {
			break;
		}
	}
	
	// forward direction(CCW 0->180) search record 
	for(int n = 0; n <= mid_num; n++) {
		if(abs(passfcn_vec_(maxfcn_bwdbnd_ + n) - max_passfcn_val_) < delta) {
			fwd_maxpass_cnt_++;
		} else {
			break;
		}
	}

	if(fwd_maxpass_cnt_ <= bwd_maxpass_cnt_) {
		angle_adj_ = maxfcn_bwdbnd_ - bwd_maxpass_cnt_ / 2.0 - mid_num;
	} else {
		angle_adj_ = maxfcn_bwdbnd_ + fwd_maxpass_cnt_ / 2.0 - mid_num;
	}
}

void APF::CalcAlarmInAPF(void) {
	if(max_passfcn_val_ <= PASSFCN_THD_RATIO * D_M) {
		apf_alarm_ = 1;
	} else {
		apf_alarm_ = 0;
	}	
}

void APF::ResetMaxPassValCnt(void) {
	fwd_maxpass_cnt_ = 0;
	bwd_maxpass_cnt_ = 0;
}

void APF::PubPfInfo4Dbg(void) {
	pf_dbg_.header.stamp = ros::Time::now();
	pf_dbg_.header.frame_id = "apf";
	pf_dbg_.angle_min = 0.0;
	pf_dbg_.angle_max = 180.0;
	pf_dbg_.angle_increment = 1.0;

	memcpy(&pf_dbg_.potential_value[0], &abstract_pf_[0], sizeof(abstract_pf_));
	pf_Pub4dbg_.publish(pf_dbg_);	
}

void APF::CalcCtrlCmd(const float & v_ref, const float & rot_coeff) {
	float tmp_rot_angle = 0.0;
	
	ctrl_v_= (v_ref - V_MIN) * (max_passfcn_val_ / D_M) + V_MIN;
		
	if(abs(angle_adj_) > 2.) {
		tmp_rot_angle = angle_adj_; //clear the quake
	}
	
	ctrl_vth_ = rot_coeff * tmp_rot_angle / 180.0;
}

void APF::ScanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_ca) {
	int j = ANGLE4CA_START_INDEX;
	for(int i = 0; i < NUM_RAY4CA; i++) {
		scan4ca_(i) = scan_ca->ranges[j];

		if(scan4ca_(i) < LASER_BLIND) {
			scan4ca_(i) = scan4ca_[i-1];
		}
		j = j + 2; 
	}
	refresh_flag_ = true;
}

void APF::EnvSecurityCallBack(const colibri_msgs::EnvSecurity::ConstPtr& env) {
	min_laser_ = env->laser_min_dis;
	min_laser_dir_ = env->laser_min_angle;
}

void APF::CalcDsrVc(const float & vel) {
	dsr_v_ = -0.5 * K_SR * vel * vel / ACC_DEC;
}

void APF::LimitAngle(Array_CA & delta_ang) {
	Array_CA right_ang;
	right_ang = 90. * right_ang.setOnes();	
	delta_ang = delta_ang.min(right_ang);
	delta_ang = delta_ang.max(-right_ang);
}

