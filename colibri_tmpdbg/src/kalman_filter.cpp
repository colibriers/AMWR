#include "kalman_filter.h"

KalmanFilter::KalmanFilter(){

	x0_ << 0.0, 0,0, 0.0;
	
	kal_state_.xhat = x0_ ;
	kal_state_.xhat_ear << 0.0, 0,0, 0.0;
	
	kal_state_.A = Matrix3d::Identity(STATE_EST_DIM, STATE_EST_DIM);
	kal_state_.B = Matrix3d::Zero();
	kal_state_.H = Matrix3d::Identity(STATE_EST_DIM, STATE_EST_DIM);
	
	Wq_ << 0.05, 0.05, 2;
	kal_state_.Q = Wq_.asDiagonal();
	
	Vr_ << 0.5, 0.5, 15;
	kal_state_.R = Vr_.asDiagonal();
	
	P0_ << 0.1, 0.1, 5.0;
	kal_state_.Phat = Matrix3d::Identity(STATE_EST_DIM, STATE_EST_DIM) * P0_; 
	kal_state_.Phat_ear = Matrix3d::Zero();
	
	kal_state_.Kg = Matrix3d::Zero(); 

	reset_kalman_ = false;
	
}

KalmanFilter::~KalmanFilter(){

}

void KalmanFilter::KalmanInit(Vector3d &x0){
	
	kal_state_.xhat = x0;
	
	Wq_ << 0.05, 0.05, 2;
	kal_state_.Q = Wq_.asDiagonal();
	
	Vr_ << 0.5, 0.5, 15;
	kal_state_.R = Vr_.asDiagonal();
	
	P0_ << 0.1, 0.1, 5.0;
	kal_state_.Phat = Matrix3d::Identity(STATE_EST_DIM, STATE_EST_DIM) * P0_; 
	kal_state_.Phat_ear = Matrix3d::Zero();
	
	kal_state_.Kg = Matrix3d::Zero(); 
	
}

void KalmanFilter::KalmanPredict(Vector3d &u){
	
	kal_state_.xhat_ear = kal_state_.A * kal_state_.xhat + kal_state_.B * u;
	kal_state_.Phat_ear = kal_state_.A * kal_state_.Phat * kal_state_.A.transpose() + kal_state_.Q;  

}

void KalmanFilter::KalmanUpdate(Vector3d &zm){

	Matrix3d tmp_PH = kal_state_.Phat_ear * kal_state_.H.transpose();

	kal_state_.Kg = tmp_PH * ((kal_state_.H * tmp_PH + kal_state_.R).inverse());
	
	kal_state_.xhat_ear = kal_state_.xhat_ear + kal_state_.Kg * (zm - kal_state_.H * kal_state_.xhat_ear);

	kal_state_.Phat = (Matrix3d::Identity(3,3) - kal_state_.Kg * kal_state_.H) * kal_state_.Phat_ear;

	x_est_ = kal_state_.xhat_ear; 
}


