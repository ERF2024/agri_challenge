#include "agri_state_machine/agri_state_machine.h"

int main(int argc, char **argv)
{

	ros::init(argc, argv, "agri_state_machine");

	AgriStateMachine ce1("left");
	AgriStateMachine ce2("right");
	ros::Rate r(500);

	while (ros::ok())
	{
		ce1.spinner();
		ce2.spinner();
		r.sleep();
	}

	return 0;
}
