#include "../include/controller.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "position_controller");
	ros::NodeHandle nh;

	// Create the controller specific to your robot
	std::shared_ptr<Controller> p_controller = std::make_shared<Controller>(nh, 500.0);
	ros::Rate r(500);

	while(ros::ok())
	{
		auto start = std::chrono::high_resolution_clock::now();
		p_controller->ComputeControlAction();
		auto end = std::chrono::high_resolution_clock::now();
		auto duration_control_action = end - start;
		
		start = std::chrono::high_resolution_clock::now();
		ros::spinOnce();
		end = std::chrono::high_resolution_clock::now();
		auto duration_spin_once = end - start;

		if(r.cycleTime() > r.expectedCycleTime() )
		{
			ROS_INFO("Cycle exceed expected cycle time: %lf ms", r.cycleTime().toSec() * 1000.0 );
			ROS_INFO("ComputeControlAction took %ld ns", duration_control_action);
			ROS_INFO("spinOnce took %ld ns", duration_spin_once);
		}
		r.sleep();
	}

	return 0;
}
