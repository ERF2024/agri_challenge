#include "agri_control/agri_control.h"

int main(int argc, char **argv)
{

	ros::init(argc, argv, "agri_control");

	ros::NodeHandle nh;
	ros::Duration(0.1).sleep();
	std::string prefix1, prefix2;
	nh.param<std::string>("prefix1", prefix1, "left");
	nh.param<std::string>("prefix2", prefix2, "right");

	AgriControl ce1(prefix1);
	AgriControl ce2(prefix2);
	ros::Rate r(500);

	while (ros::ok())
	{
		ce1.spinner();
		ce2.spinner();
		r.sleep();
	}

	return 0;
}
