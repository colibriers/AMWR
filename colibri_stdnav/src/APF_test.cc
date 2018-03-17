#include "APF_avoiding.h"

int main(int argc, char* argv[]) {

	ros::init(argc, argv, "APF_test_node");
	APF apfObj(60.);
	ros::Rate loop_rate(10);
	int delay_cnt = 0;
	
	while(ros::ok()) {
		if(delay_cnt < DELAY_CNT_MAX){
			delay_cnt++;
			ros::spinOnce();
			loop_rate.sleep();
		}

		if(delay_cnt >= DELAY_CNT_MAX){
			delay_cnt = DELAY_CNT_MAX;
		
			if(apfObj.refresh_flag_) {
		  		apfObj.CalcPhiParam(0.);
				apfObj.CalcKafTheta(70.);
				apfObj.CalcKrfTheta();
				apfObj.CalcPassFcn();
				apfObj.CalcAlarmInAPF();
				apfObj.refresh_flag_ = false;
			}
			ros::spinOnce();
			loop_rate.sleep();
		}


	}

	return 0;
}
