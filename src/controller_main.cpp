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
		double duration_control_action = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;

		start = std::chrono::high_resolution_clock::now();
		ros::spinOnce();
		end = std::chrono::high_resolution_clock::now();
		double duration_spin_once = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;

		if( r.cycleTime() > r.expectedCycleTime() )
		{
			// ROS_INFO("Cycle time exceed robot control loop");
			// ROS_INFO("Cycle time=%lf [ms], expected cycle time:%lf [ms]", r.cycleTime().toSec() * 1000.0, r.expectedCycleTime().toSec() * 1000.0 );
			// ROS_INFO("ComputeControlAction took %lf [ms]", duration_control_action);
			// ROS_INFO("spinOnce took %lf [ms]\n", duration_spin_once);
		}
		r.sleep();
	}

	return 0;
}
