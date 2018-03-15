#include "routes_smoothing.h"

template <typename T, int N = 3>
Eigen::Matrix<T, 2, 1> CalcCubicBezierValue(st_2d_point<T> *ptrPoint, T t) {
	Eigen::Matrix<T, 2, N+1> m_points;
	Eigen::Matrix<T, 2, 1> result = Eigen::MatrixXf::Zero(2, 1);
	if(ptrPoint == 0)
	{
		return result;
	}
	
	for(int j = 0; j <= N; j++) {
		m_points(0, j) = (ptrPoint + j)->x;
		m_points(1, j) = (ptrPoint + j)->y;
	}

	result = pow(1-t, 3) * m_points.col(0) + 3.0 * t * (1- t) * (1- t) * m_points.col(1) + 
						3.0 * t * t * (1- t) * m_points.col(2) + t * t * t * m_points.col(2);

	return result;
	
}

template <typename T, int N = 3>
void CubicBezierSmoothing(const  std::vector< st_2d_point<T> > &pointSeq,
																std::vector< st_2d_point<T> > &smoothSeq) {
	if(pointSeq.size() <= N)
	{
		smoothSeq(pointSeq);
		return;
	}
	st_2d_point<T> tmp;

	for(int j = 0; j < pointSeq.size() - N; j += N) {
		if( 0 == j) {
			Eigen::Matrix<T, 2, 1> dot_1 = CalcCubicBezierValue(pointSeq[j], 0.);
			tmp.x = dot_1(0, 0);
			tmp.y = dot_1(1, 0);
			smoothSeq.push_back(tmp);
		}
		
		Eigen::Matrix<T, 2, 1> dot_2 = CalcCubicBezierValue(pointSeq[j], 1/3.);
		tmp.x = dot_2(0, 0);
		tmp.y = dot_2(1, 0);
		smoothSeq.push_back(tmp);
		
		Eigen::Matrix<T, 2, 1> dot_3 = CalcCubicBezierValue(pointSeq[j], 2/3.);
		tmp.x = dot_3(0, 0);
		tmp.y = dot_3(1, 0);
		smoothSeq.push_back(tmp);
		
		Eigen::Matrix<T, 2, 1> dot_4 = CalcCubicBezierValue(pointSeq[j], 1.);
		tmp.x = dot_4(0, 0);
		tmp.y = dot_4(1, 0);
		smoothSeq.push_back(tmp);
		
	}

	return;

}

template <typename T, int M = 2, int PN = 5>
void FivePointCubicSmoothing(const std::vector< st_2d_point<T> > &pointSeq,
																		 std::vector< st_2d_point<T> > &smoothSeq) {
	std::vector< st_2d_point<T> > ().swap(smoothSeq);
	static std::vector< st_2d_point<T> > tmp_seq(pointSeq);
	int length = pointSeq.size();
	if(length <= PN) {
		smoothSeq(pointSeq);
		return;
	}

	for(int i = 0; i < M; i++) {
		tmp_seq[0].x = (69.0 * tmp_seq[0].x + 4.0 * (tmp_seq[1].x + tmp_seq[3].x) - 6 * tmp_seq[2].x - tmp_seq[4].x) / 70.;
		tmp_seq[0].y = (69.0 * tmp_seq[0].y + 4.0 * (tmp_seq[1].y + tmp_seq[3].y) - 6 * tmp_seq[2].y - tmp_seq[4].y) / 70.;
		tmp_seq[1].x = (2.0 * (tmp_seq[0].x + tmp_seq[4].x) + 27 * tmp_seq[1].x + 12 * tmp_seq[2].x + 12 * tmp_seq[2].x - 8 * tmp_seq[3].x) / 35.;
		tmp_seq[1].y = (2.0 * (tmp_seq[0].y + tmp_seq[4].y) + 27 * tmp_seq[1].y + 12 * tmp_seq[2].y + 12 * tmp_seq[2].y - 8 * tmp_seq[3].y) / 35.;
		for(int j = 2; j < (length - 2); j++ ) {
			tmp_seq[j].x = (-3.* (tmp_seq[j - 2].x + tmp_seq[j + 2].x) + 12. * (tmp_seq[j - 1].x + tmp_seq[j + 1].x) + 17. * tmp_seq[j].x) / 35.;
			tmp_seq[j].y = (-3.* (tmp_seq[j - 2].y + tmp_seq[j + 2].y) + 12. * (tmp_seq[j - 1].y + tmp_seq[j + 1].y) + 17. * tmp_seq[j].y) / 35.;
		}
		tmp_seq[length - 2].x = (2.0 * (tmp_seq[length - 1].x + tmp_seq[length - 5].x) + 27 * tmp_seq[length - 2].x + 12 * tmp_seq[length - 3].x - 8 * smoothSeq[length - 4].x) / 35.; 
		tmp_seq[length - 2].y = (2.0 * (tmp_seq[length - 1].y + tmp_seq[length - 5].y) + 27 * tmp_seq[length - 2].y + 12 * tmp_seq[length - 3].y - 8 * smoothSeq[length - 4].y) / 35.; 
		tmp_seq[length - 1].x = (69.0 * tmp_seq[length - 1].x + 4.0 * (tmp_seq[length - 2].x + tmp_seq[length - 4].x) - 6 * tmp_seq[length - 3].x - smoothSeq[length - 5].x) / 70.;
		tmp_seq[length - 1].y = (69.0 * tmp_seq[length - 1].y + 4.0 * (tmp_seq[length - 2].y + tmp_seq[length - 4].y) - 6 * tmp_seq[length - 3].y - smoothSeq[length - 5].y) / 70.;
	}

	smoothSeq(tmp_seq);
	return;
	
}

