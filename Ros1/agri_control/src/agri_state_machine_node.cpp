#include "agri_state_machine/agri_state_machine.h"

int main(int argc, char **argv)
{

	ros::init(argc, argv, "agri_state_machine");

	ros::NodeHandle nh;
	std::string prefix1, prefix2;
	nh.param<std::string>("prefix1", prefix1, "left");
	nh.param<std::string>("prefix2", prefix2, "right");
	AgriStateMachine ce1(prefix1);
	AgriStateMachine ce2(prefix2);
	ros::Rate r(500);

	while (ros::ok())
	{
		ce1.spinner();
		ce2.spinner();
		r.sleep();
	}

	return 0;
}
