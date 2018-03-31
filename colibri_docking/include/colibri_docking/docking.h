#ifndef DOCKING_H_
#define DOCKING_H_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <fstream>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#define DEG2RAD	0.0174533
#define SCAN_RAY_NUM	481

#define WIN_NUM	5
#define DELAY_CNT_MAX 10

typedef Eigen::Array<float, 1, SCAN_RAY_NUM> Array_scan;
typedef Eigen::Matrix<float, 2, SCAN_RAY_NUM> Matrix_scan;

using namespace std;
#include <iostream>
#include <string>
#include <vector>

template <class T>  
T stringToNum(const string& str);


void	ExportData(const vector<float>& data);

void SplitString(const string& s, vector<string>& v, const string& c);

struct Pose 
{
  float x;
  float y;
	 
  Pose& operator=(Pose& value) {
    x = value.x;
    y = value.y;
    return *this;
  }
  Pose operator+(Pose& value) {
		Pose result;
    result.x = x + value.x;
    result.y = y + value.y;
    return result;
  }
  Pose operator-(Pose& value) {
		Pose result;
    result.x = x - value.x;
    result.y = y - value.y;
    return result;
  }
  Pose operator*(Pose& value) {
    x *= value.x;
    y *= value.y;
    return *this;
  }

	Pose operator*(float & value) {
		Pose result;
    result.x = x * value;
    result.y = y * value;
    return result;
  }

	float Norm(void) {
		float norm = sqrt(pow(x, 2) + pow(y, 2));
		return norm;
	}

};


class ScanHandle {
	public:
		struct st_scope {
			int upper;
			int lower;
		};
		
		typedef st_scope scope;

		const int num_ = 481;
		const float resol_ = 0.5;
		const int N_min_ = 10;
		const int N_max_ = 100;
		const scope proc_domain_ = {180, -30};
		const float scan_lower_ = -30.;
		const float scan_upper_ = 210.;
		const float sin_reflect_angle = 0.707;

		std::vector<float> scan_vec_;
		Array_scan rho_origin_;
		Array_scan rho_filter_;
		Matrix_scan scan_orgin_;
		Matrix_scan scan_filter_;
		bool refresh_flag_;
		Array_scan breaker_dmax_;
		Eigen::Array<int, 1, SCAN_RAY_NUM> k_breaker_;
		std::vector<scope> segs_vec_;
		std::vector<float> ver_dis_;
		int dock_seg_index_;
		
		float max_verdis_;
		int max_verdis_index_;
		int match_corner_index_;
		int middle_index_;
		int avg_corner_index_;
		ros::NodeHandle nh_docking_;
		ros::Subscriber scan_sub4dock_;

		ScanHandle();
		~ScanHandle();
		void LoadData();
		void MedFilter(void);
		void Polar2Cartesian(const Array_scan & polar_data, Matrix_scan & cartesian_data);
		void Polar2Cartesian();
		void CalcBreakerMarker(const Array_scan & polar_data, const Matrix_scan & xy_data);
		void CalcAdaptBreakerDis(const Array_scan & polar_data);
		void CalcContiSegs(void);
		void CalcMaxDis2Segs(const Eigen::Matrix<float, 2, SCAN_RAY_NUM> & scan_xy, const scope & index);
		void CalcMaxDis2Segs(const Eigen::Matrix<float, 2, SCAN_RAY_NUM> & scan_xy, const int & seg_id); 
		void CalcMatchCornerIndex(const Eigen::Matrix<float, 2, SCAN_RAY_NUM> & scan_xy);
		void CalcCornerFunc(const Eigen::Matrix<float, 2, SCAN_RAY_NUM> & scan_xy, const int &index, const int &width);
		void CalcAvgCornerIndex(void);

	private:
		const float lamda_ = 25;
		const float sigma_ = 0.01;
		float corner_val_;
		std::vector<float> corner_vec_;
		void ScanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan);
		float CalcPoint2LineDis(Pose & a, Pose & b, Pose &c);
		
};

#endif // DOCKING_H_
