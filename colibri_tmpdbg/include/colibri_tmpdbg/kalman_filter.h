#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <cmath>
#include <Eigen/Dense>


using namespace std;
using namespace Eigen;

#define STATE_EST_DIM	3
typedef struct st_kalman{
	Vector3d xhat_ear;
	Vector3d xhat;
	Matrix3d A;
	MatrixXd B;
	Matrix3d H;
	Matrix3d Phat_ear;
	Matrix3d Phat;
	Matrix3d Q;
	Matrix3d R;
	Matrix3d Kg;

} kalman_3d_state;

class KalmanFilter{
	
	public:

		Vector3d x0_;
		Vector3d x_est_;
		
		Vector3d Wq_;
		Vector3d Vr_;
		Vector3d P0_;
		
		kalman_3d_state kal_state_;
		
		bool reset_kalman_;

		KalmanFilter();
		~KalmanFilter();

		void KalmanInit(Vector3d &x0);
		void KalmanPredict(Vector3d &u);	
		void KalmanUpdate(Vector3d &zm);

	private:
		
		
};


#endif
