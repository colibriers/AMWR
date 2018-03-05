#include "aiv_driver.h" 

bool node_shutdown = false;

void MySigintHandler(int sig) {
  node_shutdown = true;
};

int main(int argc, char* argv[]) {	
	ros::init(argc, argv, "aiv_driver_node");
	AIV_Driver driver_port;
	driver_port.InitCom("/dev/ttyS1");
	driver_port.CreateThread(ReadDataThread);

	//my_port.SendCmd(my_port.req_vel_start,AIV_Driver::req_vel_start_finish);

	if(node_shutdown == true) {
		driver_port.CloseCom("/dev/ttyS1");
	}

	ros::spin();

	return 0;	
}

