#include "docking.h"

template <class T>  
T stringToNum(const string& str)  
{  
    istringstream iss(str);  
    T num;  
    iss >> num;  
    return num;      
}  

void SplitString(const string& s, vector<string>& v, const string& c)
{
    string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while(string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2-pos1));
         
        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if(pos1 != s.length())
        v.push_back(s.substr(pos1));
}

ScanHandle::ScanHandle(): dock_seg_index_(0), max_verdis_(0.),
															max_verdis_index_(0), match_corner_index_(0) {
	//scan_sub4dock_ = nh_docking_.subscribe<sensor_msgs::LaserScan>("/scan", 1, &ScanHandle::ScanCallBack, this);
	refresh_flag_ = false;

}
ScanHandle::~ScanHandle() {

}

void	ExportData(const vector<float>& data) {

	std::string path_name;
	char user_name[10];
	getlogin_r(user_name, 10);
	std::string str_username = user_name;
	path_name.assign("/home/" + str_username + "/colibri_ws/src/colibri_docking/data/new_calc.txt");

	ofstream scan_out(path_name);
	
	for(vector<float>::const_iterator it = data.cbegin(); it != data.cend(); it++) {
		scan_out << *it << " ";	
	}
	
	scan_out.close();
}

void ScanHandle::LoadData(void) {

	std::string path_name;
	char user_name[10];
	getlogin_r(user_name, 10);
	std::string str_username = user_name;
	path_name.assign("/home/" + str_username + "/colibri_ws/src/colibri_docking/data/new.txt");

	std::ifstream scan_in(path_name.c_str());
	if(scan_in.fail()) { 
		std::cout  << "Error opening file";
		exit (1);
	} 

	string strdata;
	getline(scan_in,strdata);
	vector<string> scanfile_data_vec;
	std::vector<float> ().swap(scan_vec_);
	SplitString(strdata, scanfile_data_vec,",");
	for(vector<string>::const_iterator it = scanfile_data_vec.cbegin(); it != scanfile_data_vec.cend(); it++) {
		float tmp_scan = stringToNum<float>(*it);
		scan_vec_.push_back(tmp_scan);
	}
	
	scan_in.close();

}


void ScanHandle::MedFilter(void) {
	int filter_num = static_cast<int>((proc_domain_.upper - proc_domain_.lower) / resol_) + 1;
	int start_index = static_cast<int>((proc_domain_.lower - (scan_lower_)) / resol_);
	int half_win = static_cast<int>((WIN_NUM - 1) / 2);
	std::vector<float> win_scan_vec_(WIN_NUM);
	std::vector<float> filter_vec_(scan_vec_);

	for(int i = 0; i < filter_num - WIN_NUM ; i++) {
		win_scan_vec_.assign(scan_vec_.begin() + start_index + i , scan_vec_.begin() + start_index + i + WIN_NUM);
		std::sort(win_scan_vec_.begin(), win_scan_vec_.end());
		filter_vec_[start_index + i + half_win ] = win_scan_vec_[half_win];
		std::vector<float> ().swap(win_scan_vec_);
	}
	rho_filter_ = Eigen::Map<Array_scan>(filter_vec_.data(), filter_vec_.size());
	ExportData(filter_vec_);
}

void ScanHandle::Polar2Cartesian(const Array_scan & polar_data, Matrix_scan & cartesian_data) {
	Eigen::Array<float, 1, SCAN_RAY_NUM> index_angle, tmp_delta;
	index_angle.setLinSpaced(SCAN_RAY_NUM, scan_lower_, scan_upper_);
	Eigen::Array<float, 1, SCAN_RAY_NUM> tmp = ((DEG2RAD * index_angle).cos()) * polar_data;
	float * ptr_axis_data = tmp.data();
	cartesian_data.topRows(1) = Eigen::Map<Eigen::Matrix<float, 1, SCAN_RAY_NUM>>(ptr_axis_data);
	tmp = ((DEG2RAD * index_angle).sin()) * polar_data;
	ptr_axis_data = tmp.data();
	cartesian_data.bottomRows(1) =  Eigen::Map<Eigen::Matrix<float, 1, SCAN_RAY_NUM>>(ptr_axis_data);
}

void ScanHandle::Polar2Cartesian() {
	Eigen::Array<float, 1, SCAN_RAY_NUM> index_angle, tmp_delta;
	index_angle.setLinSpaced(SCAN_RAY_NUM, scan_lower_, scan_upper_);
	Eigen::Array<float, 1, SCAN_RAY_NUM> tmp = ((DEG2RAD * index_angle).cos()) * rho_filter_;
	float * ptr_axis_data = tmp.data();
	scan_filter_.topRows(1) = Eigen::Map<Eigen::Matrix<float, 1, SCAN_RAY_NUM>>(ptr_axis_data);
	tmp = ((DEG2RAD * index_angle).sin()) * rho_filter_;
	ptr_axis_data = tmp.data();
	scan_filter_.bottomRows(1) =  Eigen::Map<Eigen::Matrix<float, 1, SCAN_RAY_NUM>>(ptr_axis_data);
}

void ScanHandle::CalcBreakerMarker(const Array_scan & polar_data, const Matrix_scan & xy_data) {
	k_breaker_.setZero();
	std::vector<float> tmp_norm_vec;
	float tmp_norm = 0.;
	CalcAdaptBreakerDis(polar_data);
	Eigen::Array<int, 1, SCAN_RAY_NUM-1> compare;
	compare.setZero();
	Eigen::Matrix<float, 2, SCAN_RAY_NUM-1> tmp_diff = xy_data.rightCols(SCAN_RAY_NUM-1) - xy_data.leftCols(SCAN_RAY_NUM-1);
	for(int i = 0; i < SCAN_RAY_NUM-1; i++) {
		tmp_norm = tmp_diff.col(i).norm();
		compare(i) = tmp_norm > breaker_dmax_(i+1)? 1 : 0;
		if(compare(i) == 1 && i > 1) {
			compare(i-1) = 1;
		}
	}
	k_breaker_.rightCols(SCAN_RAY_NUM-1) = compare;
}

void ScanHandle::CalcAdaptBreakerDis(const Array_scan & polar_data) {
	breaker_dmax_ = sin(resol_ * DEG2RAD) * polar_data / sin((lamda_ - resol_) * DEG2RAD) + 3.0 * sigma_;
}

void ScanHandle::CalcContiSegs(void) {
	int ne = 1;
	int ni = ne;
	scope tmp_scope_element;
	std::vector<scope> ().swap(segs_vec_);
	while(ne < SCAN_RAY_NUM - 1) {
		ni = ne;
		ne = ni + 1;
		while(0 == k_breaker_(ne - 1)) {
			ne++;
			if(SCAN_RAY_NUM == ne) {
				break;
			}
		}

		if((ne - ni + 1 > N_min_) && (ne - ni + 1 < N_max_)) {
			tmp_scope_element.upper = (ne - 1) - 1;// this  -1 solve the index from 0 to keep same with matlab sim
			tmp_scope_element.lower = (ni + 1) - 1;
			segs_vec_.push_back(tmp_scope_element);	
		}
	}


	
}

void ScanHandle::CalcMaxDis2Segs(const Eigen::Matrix<float, 2, SCAN_RAY_NUM> & scan_xy, const scope & index) {
	const int cols_num = index.upper - index.lower + 1;
	Eigen::MatrixXf corner_scan_xy;
	std::vector<float> tmp_vec;
	for(int i = 0; i < cols_num; i++) {
		tmp_vec.push_back(scan_xy(0,index.lower + i));
		tmp_vec.push_back(scan_xy(1,index.lower + i));
	}
	//Eigen::MatrixXf corner_scan_xy = Eigen::Map<Eigen::Matrix<float, 2, Eigen::Dynamic >> (tmp_vec.data()); 

	//corner_scan_xy = scan_xy.block(0, index.lower, 2, cols_num);
	corner_scan_xy = scan_xy.middleCols(index.lower, cols_num);
	
	float tmp_vertical_dis = 0.;
	std::vector<float> ().swap(ver_dis_);
	Eigen::Matrix<float, 2, 1> matrix_ab = corner_scan_xy.rightCols(1) - corner_scan_xy.leftCols(1);
	Eigen::Matrix<float, 1, Eigen::Dynamic> matrix_box;
	matrix_box.setOnes();
	Eigen::Matrix<float, 2, Eigen::Dynamic> matrix_ac = corner_scan_xy - corner_scan_xy.leftCols(1) * matrix_box;
	Eigen::Matrix<float, 2, Eigen::Dynamic> matrix_ac_proj = matrix_ab / matrix_ab.norm() * (matrix_ab.transpose() * matrix_ac / matrix_ab.norm());
	Eigen::Matrix<float, 2, Eigen::Dynamic> matrix_verline = matrix_ac - matrix_ac_proj;
	for(int i = 0; i < cols_num; i++) {
	 tmp_vertical_dis = matrix_verline.col(i).norm();
	 ver_dis_.push_back(tmp_vertical_dis);
	}
	std::vector<float>::iterator biggest = std::max_element(ver_dis_.begin(), ver_dis_.end());
	max_verdis_ = *biggest;
	max_verdis_index_ = std::distance(ver_dis_.begin(), biggest);
}

float ScanHandle::CalcPoint2LineDis(Pose & a, Pose & b, Pose &c) {
	Pose vec_ab = b - a;
	Pose vec_ac = c - a;
	float norm_ab = vec_ab.Norm();
	float tmp = (vec_ab.x * vec_ac.x + vec_ab.y * vec_ac.y) / norm_ab /norm_ab;
	Pose vec_ac_proj = vec_ab * tmp;
	Pose vec_verline = vec_ac - vec_ac_proj;
	float vert_dis = vec_verline.Norm();
	return vert_dis;
}

void ScanHandle::CalcMaxDis2Segs(const Eigen::Matrix<float, 2, SCAN_RAY_NUM> & scan_xy, const int & seg_id) {
	int start = segs_vec_[seg_id].lower; 
	int terminal = segs_vec_[seg_id].upper;
	std::vector<float> ().swap(ver_dis_);
	Pose a , b, c;
	a.x = scan_xy(0, start);
	a.y = scan_xy(1, start);
	b.x = scan_xy(0, terminal);
	b.y = scan_xy(1, terminal);
	float tmp_verdis = 0.0;
	for(int i = start + 1; i <= terminal -1; i++) {
		c.x = scan_xy(0, i);
		c.y = scan_xy(1, i);		
		tmp_verdis = CalcPoint2LineDis(a, b, c);
	  ver_dis_.push_back(tmp_verdis);
	}
	std::vector<float>::iterator biggest = std::max_element(ver_dis_.begin(), ver_dis_.end());
	max_verdis_ = *biggest;
	max_verdis_index_ = std::distance(ver_dis_.begin(), biggest) + start;

}

void ScanHandle::CalcMatchCornerIndex(const Eigen::Matrix<float, 2, SCAN_RAY_NUM> & scan_xy) {
	scope dock;
	std::vector<float> ().swap(corner_vec_);
	std::vector<float> tt;
	dock.upper = segs_vec_.at(dock_seg_index_).upper;
	dock.lower = segs_vec_.at(dock_seg_index_).lower;
  int width = floor((dock.upper - dock.lower) / 4.);
	int center_index = floor((dock.upper + dock.lower) / 2.);
	float tmp_diff = 0.0;
	for(int k = center_index - width; k <=  center_index + width; k++ ) {
		CalcCornerFunc(scan_xy, k, width);
		tt.push_back(corner_val_);
		tmp_diff = abs(corner_val_ - sin_reflect_angle);
		corner_vec_.push_back(tmp_diff);
	}
	std::vector<float>::iterator smallest = std::min_element(corner_vec_.begin(), corner_vec_.end());
	match_corner_index_ = center_index - width + std::distance(corner_vec_.begin(), smallest);
}

void ScanHandle::CalcCornerFunc(const Eigen::Matrix<float, 2, SCAN_RAY_NUM> & scan_xy, const int &index, const int &width) {	
	float x = 0.;
	float y = 0.;
	for (int i = index - width; i <= index; i++) {
		x += scan_xy(0, i);
		y += scan_xy(1, i);
	}
	Eigen::Matrix<float, 2, 1> abstract_left_point;
	abstract_left_point << x/(width + 1),y/(width + 1);

	x = 0.;
	y = 0.;
	for (int i = index; i <= index + width; i++) {
		x += scan_xy(0, i);
		y += scan_xy(1, i);
	}
	Eigen::Matrix<float, 2, 1> abstract_right_point;
	abstract_right_point << x/(width + 1),y/(width + 1);
	
	float l_a = (scan_xy.col(index) - abstract_left_point).norm();
	float l_b = (scan_xy.col(index) - abstract_right_point).norm();
	float l_c = (abstract_left_point - abstract_right_point).norm();
	float l_k = (l_a + l_b + l_c) / 2.;

	corner_val_ = 2 * sqrt(l_k * (l_k - l_a) * (l_k - l_b) * (l_k - l_c)) / (l_a * l_b);
}

void ScanHandle::ScanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan) {
	std::vector<float> ().swap(scan_vec_);
	scan_vec_ = scan->ranges;
	rho_origin_ = Eigen::Map<Array_scan>(scan_vec_.data(), scan_vec_.size());
	//refresh_flag_ = true;
}


