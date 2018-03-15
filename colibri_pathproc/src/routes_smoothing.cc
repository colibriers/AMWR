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



