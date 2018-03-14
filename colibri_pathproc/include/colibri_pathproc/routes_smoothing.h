#ifndef ROUTES_SMOOTHING_H_
#define ROUTES_SMOOTHING_H_

#include <cmath>
#include <vector>
#include <Eigen/Dense>

template <typename T>
struct st_2d_point{
	T x;
	T y;
};

template <typename T, int N = 3>
Matrix<T, 2, 1> CalcCubicBezierValue(st_2d_point<T> *ptrPoint, T t);

template <typename T, int N = 3>
void CalcCubicBezierValue(const vector< st_2d_point<T> > &pointSeq,vector< st_2d_point<T> > &smoothSeq);



#endif