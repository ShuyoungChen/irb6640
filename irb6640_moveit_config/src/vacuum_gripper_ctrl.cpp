#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>

class Vacuum_gripper {

	public:
		Vacuum_gripper();

	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

		ros::NodeHandle nh_;

		std_srvs::Empty srv_;
		ros::Subscriber sub_; 
		ros::ServiceClient client_on_, client_off_;
};

Vacuum_gripper::Vacuum_gripper() {
	sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 100, &Vacuum_gripper::joyCallback, this);
	client_on_ = nh_.serviceClient<std_srvs::Empty>("irb6640/vacuum_gripper/on");
	client_off_ = nh_.serviceClient<std_srvs::Empty>("irb6640/vacuum_gripper/off");
};

void Vacuum_gripper::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {   
	if (joy->axes[4] == 1) {
		client_on_.call(srv_);
		std::cout << "turning on vacuum gripper..." << std::endl;
	}
	else if (joy->axes[4] == -1) {
		client_off_.call(srv_);
		std::cout << "turning off vacuum gripper..." << std::endl;
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "vacuum_gripper_ctrl");

	Vacuum_gripper v;
	ros::spin();
}

