#include "ros/ros.h"
#include "kalman_filter.h"
#include "kalman_interface.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Kalman_Filter");
	ROS_INFO("Start to kalman filter ... ");

	KalmanInterface kalmanObj;
	KalmanFilter kalFilterObj;

	Vector3d x_0(0.0, 0.0, 0.0);
	Vector3d u(0.0, 0.0, 0.0);
	Vector3d zm(0.0, 0.0, 0.0);

	ros::Rate loop_rate(10);

	while(ros::ok()){
		
		if(kalmanObj.init_flag == true){
			
			kalmanObj.init_flag = false;
			x_0(0) = kalmanObj.set_init_state[0];
			x_0(1) = kalmanObj.set_init_state[1];
			x_0(2) = kalmanObj.set_init_state[2];
			kalFilterObj.KalmanInit(x_0);
		}

		if(kalmanObj.amcl_flag == true){
			
			zm(0) = kalmanObj.amcl_cur_state[0];
			zm(1) = kalmanObj.amcl_cur_state[1];
			zm(2) = kalmanObj.amcl_cur_state[2];
		}

		kalFilterObj.KalmanPredict(u);
		
		kalFilterObj.KalmanUpdate(zm);

		cout<<"kalFilterObj.x_est: "<<kalFilterObj.x_est_<<endl;

		loop_rate.sleep();
		
	}
	
	return 0;
}



