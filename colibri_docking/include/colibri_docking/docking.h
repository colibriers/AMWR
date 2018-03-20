#ifndef DOCKING_H_
#define DOCKING_H_

#include <cmath>
#include <iostream>
#include <vector>

#include <eigen3/Eigen/Core>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#define DEG2RAD	0.017477
#define SCAN_RAY_NUM	481

class Scan_Handle {
	public:
		struct st_scope {
			int upper;
			int lower;
		}scope;

		typedef Eigen::Array<float, 1, SCAN_RAY_NUM> Array_scan;
		typedef Eigen::Matrix<float, 2, SCAN_RAY_NUM> Matrix_scan;
		
		const int num_;
		const float resol_;
		const int win_num_;
		const int N_min_;
		const int N_max_;
		const scope proc_domain_;
		Array_scan rho_origin_;
		Array_scan rho_filter_;
		Matrix_scan scan_orgin_;
		Matrix_scan scan_filter_;

		void MedFilter(void);
		void CalcBreakerMarker(void);
		void CalcAdaptBreakerDis(void);
		void CalcContiSegs(void);
		void CalcMaxDis2Segs(void);
		float CalcCornerFunc(void);
		


};




#endif // DOCKING_H_
