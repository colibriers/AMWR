#include "task_mgr.h"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "read_tasklist_node");
	task_mgr taskObj;
	
	taskObj.ReadTasklist();

	taskObj.InitGoalMarkers(&taskObj.goalmark_list);


	ros::Rate loop_rate(5);
	while(ros::ok())
	{
		taskObj.pub_goal_maker.publish(taskObj.goalmark_list);

		cout<<"taskObj.cur_goal[0]: "<<taskObj.cur_goal[0]<<endl;
		cout<<"taskObj.cur_goal[1]: "<<taskObj.cur_goal[1]<<endl;
		cout<<"taskObj.cur_goal[2]: "<<taskObj.cur_goal[2]<<endl;

	
		ros::spinOnce();
		loop_rate.sleep();
		cout<<"End..................................."<<endl;
	}


	return 0;

}






