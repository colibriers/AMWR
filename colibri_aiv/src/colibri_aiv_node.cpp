#include "aiv_driver.h" 

bool node_shutdown  = false;
void MySigintHandler(int sig);

int main(int argc, char* argv[])
{	
	ros::init(argc, argv, "aiv_driver_node");
	AIV_Driver my_port;
	my_port.InitCom("/dev/ttyS1");
	my_port.CreateThread(ReadDataThread);
	
	//my_port.SendCmd(my_port.enable_motor,AIV_Driver::enable_motor_finish);
	//my_port.SendCmd(my_port.req_vel_start,AIV_Driver::req_vel_start_finish);

	//my_port.SendCmd(my_port.req_encoder_start,AIV_Driver::req_encoder_start_finish);

	//my_port.SendCmd(my_port.req_ultra_start,AIV_Driver::req_ultra_start_finish);

	//my_port.SendCmd(my_port.req_bumper_start,AIV_Driver::req_bumper_start_finish);

	if(node_shutdown == true)
	{
		my_port.CloseCom("/dev/ttyS1");
	}
	
	ros::spin();

	return 0;	
}

void MySigintHandler(int sig)
{
	node_shutdown = true;
}


