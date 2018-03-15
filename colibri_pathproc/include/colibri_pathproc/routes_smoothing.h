#ifndef ROUTES_SMOOTHING_H_
#define ROUTES_SMOOTHING_H_

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <vector>

template <typename T>
struct st_2d_point{
	T x;
	T y;
};

template <typename T, int N = 3>
Eigen::Matrix<T, 2, 1> CalcCubicBezierValue(st_2d_point<T> *ptrPoint, T t);

template <typename T, int N = 3>
void CubicBezierSmoothing(const std::vector< st_2d_point<T> > &pointSeq, std::vector< st_2d_point<T> > &smoothSeq);

template <typename T, int M = 2, int PN = 5>
void FivePointCubicSmoothing(const std::vector< st_2d_point<T> > &pointSeq, std::vector< st_2d_point<T> > &smoothSeq);

#endif