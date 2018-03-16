#include "APF_avoiding.h"

APF::scan_ca()
{
	goal_dir = 60;	//goal direction
	memset(scan4ca, 0.2, NUM_RAY4CA);
	ptrScan4ca = scan4ca;

	memset(ultra4ca, 20.0, NUM_RAY4CA);
	memset(add_obs4ca, 20.0, NUM_RAY4CA);
	memset(abstract_pf, 20.0, NUM_RAY4CA);
	

	
	memset(delta_phi_vec, 0, NUM_RAY4CA);
	memset(kp_phi_vec, 0, NUM_RAY4CA);

	memset(phi_start_vec, 1, NUM_RAY4CA);
	memset(phi_end_vec, 181, NUM_RAY4CA);

	memset(krf_vec, 1/D_M, NUM_RAY4CA);
	memset(kaf_vec, 0.0, NUM_RAY4CA);

	memset(passfcn_vec, 0, NUM_RAY4CA);
	max_passfcn_val = 0.0;

	min_ultra = 3.0;
	ultra_coder = 0;
	min_laser = 20.0;
	min_laser_dir = 90;

	fwd_maxpass_cnt = 0;
	bwd_maxpass_cnt = 0;

	maxfcn_fwdbnd = 0;
	maxfcn_bwdbnd = 0;

	v = 0.0;
	vth = 0.0;
	
	angle_adj = 0.0;
	vel_adj = 0.0;
	apf_alarm = 0;

	wander = 0;
	
	scan_sub4ca = nh_ca.subscribe<sensor_msgs::LaserScan>("/scan", 1, &scan_ca::ScanCallBack, this);
	env_sub4safe = nh_ca.subscribe<colibri_msgs::EnvSecurity>("/env_secure", 1, &scan_ca::EnvSecurityCallBack,this);
	//apf_pub4mntr = nh_ca.advertise<colibri_msgs::AngPotnEngy>("/apf", 5);
	//rf_pub4mntr = nh_ca.advertise<colibri_msgs::AngPotnEngy>("/rf", 5);
	pf_Pub4dbg = nh_ca.advertise<colibri_msgs::AngPotnEngy>("/pf_dbg", 5); 
}

APF::~scan_ca()
{
	
}

void APF::ScanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_ca)
{
	int j = 61;
	for(int i = 0; i < NUM_RAY4CA; i++)
	{
		scan4ca[i] = scan_ca->ranges[j];
/*		if(scan4ca[i] < 0.1)
		{
			scan4ca[i] = (scan_ca->ranges[j-2]+scan_ca->ranges[j+2]+scan_ca->ranges[j-5]+scan_ca->ranges[j+5]) / 4 + LASER_EDGE_MIN;
		}
*/
		if(scan4ca[i] < 0.05)
		{
			scan4ca[i] = scan4ca[i-1];
		}
		j = j + 2; 

		add_obs4ca[i] = 20.0;
	
	}
	
}
void APF::TrimLaserRange4CA(float & compensate)
{
	int half_width = 0;

	if(min_laser < LASER4CA_FILL_DIS)
	{
		if(min_laser < 0.25)
		{
			return;
		}
		half_width = (int) (LASER4CA_FILL_DIS / min_laser * compensate / 2.0);
		for(int i = min_laser_dir - half_width; i <= (min_laser_dir + half_width); i++)
		{
			if((i > 0) && (i < NUM_RAY4CA))
			{
				add_obs4ca[i] = min_laser;
			}
			else
			{

			}
				
		}
	}
	

}

void APF::EnvSecurityCallBack(const colibri_msgs::EnvSecurity::ConstPtr& env)
{
	min_laser = env->laser_min_dis;
	min_laser_dir = env->laser_min_angle;

}


void APF::CalcKrfTheta(float* ptrKp_phi_vector, int* ptrPhi_range_start, int* ptrPhi_range_end)
{

	for(int theta_index = 0; theta_index < NUM_RAY4CA; theta_index++)
	{
		for(int phi_index = 0; phi_index < NUM_RAY4CA; phi_index++)
		{
			if((theta_index >= *(ptrPhi_range_start+phi_index)) && (theta_index <= *(ptrPhi_range_end+phi_index)))
			{
				if(krf_vec[theta_index] <= *(ptrKp_phi_vector + phi_index)) // to obtain the max [Krf(phi,theta)] as Krf(theta)
				{	
					krf_vec[theta_index] = *(ptrKp_phi_vector + phi_index);				
				}

			}

			
		}
	}
	
}

void APF::CalcPhiRange(int i, int range_num, int* ptrPhi_start, int* ptrPhi_end)
{
	int index_start = 0;
	int index_end = 0;

	index_start = i - range_num;
	index_end = i + range_num;

	if(index_start < 1)
	{
		*ptrPhi_start = 1;
		*ptrPhi_end = index_end;
	}else if(index_end > NUM_RAY4CA)
	{
		*ptrPhi_start = index_start;
		*ptrPhi_end = NUM_RAY4CA;				
	}else
	{
		*ptrPhi_start = index_start;
		*ptrPhi_end = index_end;
	}

}

void APF::CalcPassFcnAndFwdBnd(unsigned int flag, float* max_passfcn_val, float* ptrK_pg)
{
	float tmp_passfcn_value = 0.0;
	int tmp_bound_forward = 0;
		
		for(int j = 0; j < NUM_RAY4CA; j++) //from right to left search the max passfcn val  and the max passfcn's bound index should locate at left terminal
		{
			if(flag == 1)
			{
				*(ptrK_pg + j) = 1 / (krf_vec[j]);
			}
			else
			{
				*(ptrK_pg + j) = kaf_vec[j] / (krf_vec[j]);
				abstract_pf[j] = 1 / (krf_vec[j]);
			}
		
			if(tmp_passfcn_value <= *(ptrK_pg + j))
			{
				tmp_bound_forward = j;
				tmp_passfcn_value = *(ptrK_pg + j);
			}

			
		}
		
		maxfcn_fwdbnd = tmp_bound_forward;
		*max_passfcn_val = tmp_passfcn_value;

}

void APF::CalcPassFcnWithoutRPF(float* max_passfcn_val, float* ptrK_pg, float* heading)
{	
	vector<float> tmp_apf;
	
	for(int j = 0; j < NUM_RAY4CA; j++) //from right to left search the max passfcn val  and the max passfcn's bound index should locate at left terminal
	{

		*(ptrK_pg + j) = D_M * kaf_vec[j] ;
		tmp_apf.push_back(kaf_vec[j]);
	}

	std::vector<float>::iterator biggest = std::max_element(tmp_apf.begin(), tmp_apf.end());  
	*heading = float(std::distance(tmp_apf.begin(), biggest)) - 90.0;  
	 
	*max_passfcn_val = D_M * (*biggest);
}

void APF::CalcPassFcnAndBwdBnd(unsigned int flag, float* max_passfcn_val, float* ptrK_pg)
{
	float tmp_passfcn_value = 0.0;
	int tmp_bound_backward = 0;

	for(int k = NUM_RAY4CA - 1; k >= 0; k--) //from left to right search the max passfcn val  and the max passfcn's bound index should locate at right terminal
	{
		if(flag == 1)
		{
			*(ptrK_pg + k) = 1 / (krf_vec[k]);
		}
		else
		{
			*(ptrK_pg + k) = kaf_vec[k] / (krf_vec[k]);
		}		
		if(tmp_passfcn_value <= *(ptrK_pg + k))
		{
			tmp_bound_backward = k;
			tmp_passfcn_value = *(ptrK_pg + k);
		}	
	}
		
	maxfcn_bwdbnd = tmp_bound_backward;
	*max_passfcn_val = tmp_passfcn_value;

}

void APF::CalcAlarmInAPF(void)
{
	if(max_passfcn_val <= PASSFCN_THD_RATIO * D_M)
	{
		apf_alarm = 1;
	}
	else
	{
		apf_alarm = 0;
	}	
}

float APF::CalcAdjDir(float* ptrPassfcn_vector, float max_passfcn_val, int* fwd_bound, int* bwd_bound)
{
	int mid_num = 90;
	float adj_dir = 0.0;

	float tmp_scope = MIN(PASSVAL_TOLLERENCE * max_passfcn_val, MAX_PASSFCN_SCOPE);
	
	for(int m = 0; m <= mid_num; m++)  //backward directrion(CW 180->0) search using forward bound
	{
		if(abs(*(ptrPassfcn_vector + *(fwd_bound) - m) - max_passfcn_val) < tmp_scope)
		{
			fwd_maxpass_cnt++;
		}else
		{
			break;
		}
	}
	for(int n = 0; n <= mid_num; n++)	// forward direction(CCW 0->180) search using backward bound
	{
		if(abs(*(ptrPassfcn_vector + *(bwd_bound) + n) - max_passfcn_val) < tmp_scope)
		{
			bwd_maxpass_cnt++;
		}else
		{
			break;
		}
	}

	
	if (fwd_maxpass_cnt <= bwd_maxpass_cnt)
	{
		adj_dir = *bwd_bound + bwd_maxpass_cnt / 2.0 - mid_num;
	}else
	{
		adj_dir = *fwd_bound - fwd_maxpass_cnt / 2.0 - mid_num;
	}

	return adj_dir;

}
		
float APF::CalcDsrVc(float vel_center)
{
	float d_sr_v = 0.0;
	
	d_sr_v = -0.5 * K_SR * vel_center * vel_center / ACC_DEC;

	return d_sr_v;
}

float APF::CalcKpPhi(float vel_center, float d_phi)
{
	float tmpDsr = 0.0;
	float kp_phi = 0.0;
	tmpDsr = CalcDsrVc(vel_center);

	if(d_phi <= tmpDsr)
	{
		kp_phi = KP_PHI_INF;
	}
	else if((d_phi <= D_M)&&(tmpDsr < d_phi))
	{
		kp_phi = 1 / (d_phi - tmpDsr);
	}
	else if(d_phi > D_M)
	{
		kp_phi = 1 / (D_M - tmpDsr);
	}
	else
	{
		cout<<"Calc Kp_phi exception in CalcKpPhi Function ! : " <<endl;
		cout<< "tmpDsr : "<< tmpDsr <<endl;
		cout<< "d_phi : "<< d_phi <<endl;
	}
	
	return kp_phi;
}

void APF::ResetMaxPassValCnt(void)
{
	fwd_maxpass_cnt = 0;
	bwd_maxpass_cnt = 0;
}

void scan_ca::LimitAngle(float & delta_ang)
{
	if(delta_ang >= 90) // for calc attract field , the delta angle should be -90~90 deg to ignore neg val
	{
		delta_ang = 90;
	}
	else if (delta_ang <= -90)
	{
		delta_ang = -90;
	}
	else
	{

	}
}

void APF::CalcCorrectedKrf(void)
{
	float corrector;
	
	for(int i = 0; i < NUM_RAY4CA; i++)
	{
		corrector = CalcKrfCorrectFactor(i);
		krf_vec[i] = corrector * krf_vec[i];
	}	
}

float APF::CalcKrfCorrectFactor(int index)
{
	//to make the scan front circle krf_vec factor as : y = 1+k * pow(x,2)
	float factor = 1.0;
	float tmp_puv = (90 - index) / 90.0; 	// index : 0~180
	
	factor = 1.0 + KRF_CORRECTOR * pow(tmp_puv, 2);

	return factor;
}

void APF::PubPfInfo4Dbg(void)
{
	pf_dbg.header.stamp = ros::Time::now();
	pf_dbg.header.frame_id = "apf";
	pf_dbg.angle_min = 0.0;
	pf_dbg.angle_max = 180.0;
	pf_dbg.angle_increment = 1.0;

	memcpy(&pf_dbg.potential_value[0], &abstract_pf[0], sizeof(abstract_pf));
	pf_Pub4dbg.publish(pf_dbg);
			

}
void APF::CalcPhiParam(float vel_center, float& dir_goal_inlaser)
{

	int range_num = 0;
	float tmp_delta = 0.0;
	float min_multi_range = 20.0;

#ifdef ULTRA_RF	
	float min_ultra_dis = 3.0;
	int ultra_obs_coder = 0;
	int ultra_strategy = 0;

	
#endif

#ifdef EXT_LASER_RF
	float compensate = 10.0;
	TrimLaserRange4CA(compensate);
#endif

	for(int i = 0; i < NUM_RAY4CA; i++)
	{

#ifdef ORI_ULTRA_FUSION
		min_multi_range = MIN(*(ptrScan4ca + i), ultra4ca[i]);
		delta_phi_vec[i] = asin(D_SF / min_multi_range) * RAD2DEG; //calc the phi ang obs influence range
#endif

#ifdef ORI_EXTLASER_FUSION
		min_multi_range = MIN(*(ptrScan4ca + i), add_obs4ca[i]);
		delta_phi_vec[i] = asin(D_SF / min_multi_range) * RAD2DEG; //calc the phi ang obs influence range
#endif

#ifdef ORI_EXT_ULTRA_FUSION
		min_multi_range = MIN(*(ptrScan4ca + i), add_obs4ca[i]);
		min_multi_range = MIN(min_multi_range, ultra4ca[i]);
		delta_phi_vec[i] = asin(D_SF / min_multi_range) * RAD2DEG; //calc the phi ang obs influence range
#endif


#ifdef NO_FUSION
		delta_phi_vec[i] = asin(D_SF / (*(ptrScan4ca + i))) * RAD2DEG;
#endif

		range_num = floor(delta_phi_vec[i] / RAY_RESOL4CA);
		CalcPhiRange(i, range_num, &phi_start_vec[i], &phi_end_vec[i]);	

		kp_phi_vec[i] = CalcKpPhi(vel_center, *(ptrScan4ca + i));		
		krf_vec[i] = kp_phi_vec[i];
	
		tmp_delta = i - dir_goal_inlaser;			// unit in degree
		LimitAngle(tmp_delta);
		kaf_vec[i] = cos(tmp_delta * DEG2RAD);  //dir_goal_in_laser is 0~180 degree from right side
		
	}

}




