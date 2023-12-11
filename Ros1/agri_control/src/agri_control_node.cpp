#include "agri_control/agri_control.h"

int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "agri_control");

	AgriControl ce;
	ros::Rate r(500);

	while(ros::ok())
	{
		ce.spinner();
		r.sleep();
	}

	return 0;

}
