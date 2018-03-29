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
	scan_sub4dock_ = nh_docking_.subscribe<sensor_msgs::LaserScan>("/scan", 1, &ScanHandle::ScanCallBack, this);
	refresh_flag_ = false;

}
ScanHandle::~ScanHandle() {

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
	std::vector<float> tmp_scan_vec_(scan_vec_);

	for(int i = 0; i < filter_num - WIN_NUM; i++) {
		std::sort(tmp_scan_vec_.begin() + start_index + i, tmp_scan_vec_.begin()+ start_index + WIN_NUM + i);
	}
	rho_filter_ = Eigen::Map<Array_scan>(tmp_scan_vec_.data(), tmp_scan_vec_.size());
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
			tmp_scope_element.upper = ne - 1;
			tmp_scope_element.lower = ni + 1;
			segs_vec_.push_back(tmp_scope_element);
		}
	}
}

void ScanHandle::CalcMaxDis2Segs(const Eigen::Matrix<float, 2, Eigen::Dynamic> & scan_xy) {
	const int cols_num = scan_xy.cols();
	float tmp_vertical_dis = 0.;
	std::vector<float> ().swap(ver_dis_);
	Eigen::Matrix<float, 2, 1> matrix_ab = scan_xy.rightCols(1) - scan_xy.leftCols(1);
	Eigen::Matrix<float, 1, Eigen::Dynamic> matrix_box;
	matrix_box.setOnes();
	Eigen::Matrix<float, 2, Eigen::Dynamic> matrix_ac = scan_xy - scan_xy.leftCols(1) * matrix_box;
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

void ScanHandle::CalcMatchCornerIndex(const Eigen::Matrix<float, 2, SCAN_RAY_NUM> & scan_xy) {
	scope dock;
	std::vector<float> ().swap(corner_vec_);
	dock.upper = segs_vec_.at(dock_seg_index_).upper;
	dock.lower = segs_vec_.at(dock_seg_index_).lower;
  int width = floor((dock.upper - dock.lower) / 4.);
	int center_index = floor((dock.upper + dock.lower) / 2.);
	for(int k = center_index - width; k <=  center_index + width; k++ ) {
		CalcCornerFunc(scan_xy, k, width);
		array_corner_ << corner_val_;
	}
	
	array_corner_ = (array_corner_ - 0.707).abs();
	int i = 0;
  array_corner_.minCoeff(&i, &match_corner_index_);
	
}

void ScanHandle::CalcCornerFunc(const Eigen::Matrix<float, 2, SCAN_RAY_NUM> & scan_xy, const int &index, const int &width) {
	Eigen::Matrix<float, 2, Eigen::Dynamic> tmp_matrix = scan_xy.middleCols(index - width, width + 1);
	Eigen::Matrix<float, 2, 1> abstract_left_point =tmp_matrix.rowwise().sum() / (width + 1);
	tmp_matrix = scan_xy.middleCols(index, width + 1);
	Eigen::Matrix<float, 2, 1> abstract_right_point = tmp_matrix.rowwise().sum() / (width + 1);

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


