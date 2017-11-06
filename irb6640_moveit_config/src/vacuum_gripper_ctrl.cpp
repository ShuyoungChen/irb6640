#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>

class Vacuum_gripper {

	public:
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {   
			if (joy->axes[4] == 1) {
				client.call(srv);
				std::cout << "turning on vacuum gripper..." << std::endl;
			}
			else if (joy->axes[4] == -1) {
				client_off.call(srv);
				std::cout << "turning off vacuum gripper..." << std::endl;
			}
		}


	protected:
		ros::NodeHandle n_;
		std_srvs::Empty srv;
		ros::Subscriber sub; 
		ros::ServiceClient client = n_.serviceClient<std_srvs::Empty>("irb6640/vacuum_gripper/on");
		ros::ServiceClient client_off = n_.serviceClient<std_srvs::Empty>("irb6640/vacuum_gripper/off");
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "vacuum_gripper_ctrl");
	ros::NodeHandle nh;
	ros::Subscriber sub;

	Vacuum_gripper v;
	sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1000, &Vacuum_gripper::joyCallback, &v);
	ros::spin();
}






