#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

geometry_msgs::Pose pose;

bool empty;  // whether the wrist camera can track any ar tags, if not, use the left overhead camera
bool attached = false;  // or publish to a topic

// Callback function for vision servo
/******* Convert roll, pith, yaw and quaternions. *******/

/*tf::Quaternion q(req.markers[0].pose.pose.orientation.x, 
  req.markers[0].pose.pose.orientation.y,
  req.markers[0].pose.pose.orientation.z,
  req.markers[0].pose.pose.orientation.w);

  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
 */


void alvarCallback_2(boost::shared_ptr<ar_track_alvar_msgs::AlvarMarkers const> msg, int marker_number) {   
	geometry_msgs::Pose p;

	p = msg->markers[marker_number].pose.pose;
	tf::TransformListener listener;
	tf::StampedTransform transform;
	tf::Transform tr, new_pose;

	tf::Quaternion q_prim = tf::Quaternion(p.orientation.x,
			p.orientation.y,
			p.orientation.z,
			p.orientation.w) * tf::Quaternion(1, 0, 0, 0) * tf::Quaternion(0, 0, -0.707, 0.707);


	tr.setRotation(tf::Quaternion(q_prim.x(),
				q_prim.y(),
				q_prim.z(),
				q_prim.w()));
	tr.setOrigin(tf::Vector3(p.position.x, p.position.y, p.position.z-0.6));

	if (abs(p.position.z - 0.6) < 0.05) 
		attached = false;

	ros::Time now = ros::Time(0);
	listener.waitForTransform("/base_link", "/world_camera_link_optical_2", now, ros::Duration(3.0));
	listener.lookupTransform("/base_link", "/world_camera_link_optical_2", now, transform);

	new_pose = transform * tr;

	// Construct a target frame for visualization in Rviz
	tf::TransformBroadcaster br;
	br.sendTransform(tf::StampedTransform(new_pose, now, "world", "target"));

	pose.position.x = new_pose.getOrigin().x();
	pose.position.y = new_pose.getOrigin().y();
	pose.position.z = new_pose.getOrigin().z();

	//pose.orientation.x = new_pose.getRotation().x();
	//pose.orientation.y = new_pose.getRotation().y();
	//pose.orientation.z = new_pose.getRotation().z();
	//pose.orientation.w = new_pose.getRotation().w(); 

	pose.orientation.x = 0.0;
	pose.orientation.y = 1.0;
	pose.orientation.z = 0.0;
	pose.orientation.w = 0.0; 
}

void alvarCallback_3(boost::shared_ptr<ar_track_alvar_msgs::AlvarMarkers const> msg) {   
	ROS_INFO("Move to the target position to place the panel...");
	geometry_msgs::Pose p;

	p = msg->markers[0].pose.pose;

	tf::TransformListener listener;
	tf::StampedTransform transform;
	tf::Transform tr, new_pose;

	tf::Quaternion q_prim = tf::Quaternion(p.orientation.x,
			p.orientation.y,
			p.orientation.z,
			p.orientation.w) * tf::Quaternion(1, 0, 0, 0) * tf::Quaternion(-0.707, 0, 0.707, 0);


	tr.setRotation(tf::Quaternion(q_prim.x(),
				q_prim.y(),
				q_prim.z(),
				q_prim.w()));
	tr.setOrigin(tf::Vector3(p.position.x, p.position.y, p.position.z-0.6));

	ros::Time now = ros::Time(0);
	listener.waitForTransform("/base_link", "/world_camera_link_optical_3", now, ros::Duration(3.0));
	listener.lookupTransform("/base_link", "/world_camera_link_optical_3", now, transform);

	new_pose = transform * tr;

	// Construct a target frame for visualization in Rviz
	tf::TransformBroadcaster br;
	br.sendTransform(tf::StampedTransform(new_pose, now, "world", "target"));

	pose.position.x = new_pose.getOrigin().x();
	pose.position.y = new_pose.getOrigin().y();
	pose.position.z = new_pose.getOrigin().z();

	//pose.orientation.x = new_pose.getRotation().x();
	//pose.orientation.y = new_pose.getRotation().y();
	//pose.orientation.z = new_pose.getRotation().z();
	//pose.orientation.w = new_pose.getRotation().w(); 

	pose.orientation.x = 0.0;
	pose.orientation.y = 0.707;
	pose.orientation.z = 0.0;
	pose.orientation.w = -0.707; 
}

void alvarCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {   
	empty = msg->markers.empty();
	if (empty) {
		ROS_INFO("The wrist camera cannot see the tag...");
	}
	else {
		empty = false;

		ROS_INFO("The wrist camera can see the tag..");
		geometry_msgs::Pose p;
		p = msg->markers[0].pose.pose;
		if (abs(p.position.z - 0.5) < 0.05) {
			attached = true;
			boost::shared_ptr<ar_track_alvar_msgs::AlvarMarkers const> pt;
			// subscribe the topic once and then shutdown
			pt = ros::topic::waitForMessage<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker_2", ros::Duration(5));

			while (!pt) {
				pt = ros::topic::waitForMessage<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker_2", ros::Duration(5));
			}
			while(attached) {
				alvarCallback_2(pt, 0);
			}
		}

		tf::TransformListener listener;
		tf::StampedTransform transform, transform_1;
		tf::Transform tr, new_pose;

		tf::Quaternion q_prim = tf::Quaternion(p.orientation.x,
				p.orientation.y,
				p.orientation.z,
				p.orientation.w) * tf::Quaternion(1, 0, 0, 0) * tf::Quaternion(0, 0, -0.707, 0.707);


		tr.setRotation(tf::Quaternion(q_prim.x(),
					q_prim.y(),
					q_prim.z(),
					q_prim.w()));
		tr.setOrigin(tf::Vector3(p.position.x, p.position.y, p.position.z-0.5));

		ros::Time now = ros::Time(0);
		listener.waitForTransform("/base_link", "/camera_link_optical", now, ros::Duration(3.0));
		listener.lookupTransform("/base_link", "/camera_link_optical", now, transform);

		new_pose = transform * tr;

		// Construct a target frame for visualization in Rviz
		tf::TransformBroadcaster br;
		br.sendTransform(tf::StampedTransform(new_pose, now, "world", "target"));

		pose.position.x = new_pose.getOrigin().x();
		pose.position.y = new_pose.getOrigin().y();
		pose.position.z = new_pose.getOrigin().z();
		pose.orientation.x = new_pose.getRotation().x();
		pose.orientation.y = new_pose.getRotation().y();
		pose.orientation.z = new_pose.getRotation().z();
		pose.orientation.w = new_pose.getRotation().w();  
	}
}


// Callback function for force control
//void forceCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_planning");
	ros::NodeHandle node_handle;

	// subscriber for vision topic
	/*ros::CallbackQueue vision_queue;
	  ros::SubscribeOptions opt = ros::SubscribeOptions::create<ar_track_alvar_msgs::AlvarMarkers>(
	  "/ar_pose_marker_1",
	  100,
	  &alvarCallback,
	  ros::VoidPtr(),
	  &vision_queue);
	  ros::Subscriber vision_sub = node_handle.subscribe(opt); 

	// Deal with thread block
	ros::AsyncSpinner spinner(1, &vision_queue);
	spinner.start();*/
	ros::AsyncSpinner mainspinner(1);
	mainspinner.start();

	static const std::string PLANNING_GROUP = "Manipulator";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	// Raw pointers are frequently used to refer to the planning group for improved performance.
	//const robot_state::JointModelGroup *joint_model_group =
	//move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
	visual_tools.deleteAllMarkers();

	visual_tools.loadRemoteControl();

	// Rviz provides many types of markers, in this demo we will use text, cylinders, and spheres
	/*Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
	  text_pose.translation().z() = 1.75; // above head of PR2
	  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
	 */
	// Batch publishing is used to reduce the number of messages being sent to Rviz for large visualizations
	visual_tools.trigger();

	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;

	ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
	ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

	move_group.setGoalPositionTolerance(0.004);
	move_group.setGoalOrientationTolerance(0.01);
	move_group.allowReplanning(true);

	/*Plan one*/
	boost::shared_ptr<ar_track_alvar_msgs::AlvarMarkers const> pt;
	// subscribe the topic once and then shutdown
	while (!pt) {
		pt = ros::topic::waitForMessage<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker_3", ros::Duration(5));
	}

	if (!pt->markers.empty()) {
		alvarCallback_3(pt);
	}

	//while (ros::ok()) {
	move_group.setPoseTarget(pose);  
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success = move_group.plan(my_plan);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
	if (success) {
		move_group.execute(my_plan);
		std::cout << "executing..." << std::endl;
	}
	else
		ROS_ERROR("Cannot find a plan.");

	//ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
	//visual_tools.publishAxisLabeled(pose, "pose1");

	ros::Duration(2).sleep();

	/*Plan two*/
	move_group.clearPoseTargets();
	boost::shared_ptr<ar_track_alvar_msgs::AlvarMarkers const> pt2;
	while (!pt2) {
		pt2 = ros::topic::waitForMessage<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker_2", ros::Duration(5));
	}

	if (!pt2->markers.empty()) {
		alvarCallback_2(pt2, 0);
	}

	move_group.setPoseTarget(pose);  
	moveit::planning_interface::MoveGroupInterface::Plan my_plan_2;
	bool success_2 = move_group.plan(my_plan_2);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal) %s", success_2 ? "" : "FAILED");
	if (success_2) {
		move_group.execute(my_plan_2);
		std::cout << "executing..." << std::endl;
	}
	else
		ROS_ERROR("Cannot find a plan.");

	//ROS_INFO_NAMED("tutorial", "Visualizing plan 2 as trajectory line");
	//visual_tools.publishAxisLabeled(pose, "pose2");

	ros::Duration(2).sleep();

	/*Plan three*/
	move_group.clearPoseTargets();
	boost::shared_ptr<ar_track_alvar_msgs::AlvarMarkers const> pt3;
	while (!pt3) {
		pt3 = ros::topic::waitForMessage<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker_3", ros::Duration(5));
	}

	if (!pt3->markers.empty()) {
		alvarCallback_3(pt3);
	}

	move_group.setPoseTarget(pose);  
	moveit::planning_interface::MoveGroupInterface::Plan my_plan_3;
	bool success_3 = move_group.plan(my_plan_3);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (pose goal) %s", success_3 ? "" : "FAILED");
	if (success_3) {
		move_group.execute(my_plan_3);
		std::cout << "executing..." << std::endl;
	}
	else
		ROS_ERROR("Cannot find a plan.");

	//ROS_INFO_NAMED("tutorial", "Visualizing plan 3 as trajectory line");
	//visual_tools.publishAxisLabeled(pose, "pose3");

	ros::Duration(2).sleep();

	/*Plan four*/
	move_group.clearPoseTargets();
	boost::shared_ptr<ar_track_alvar_msgs::AlvarMarkers const> pt4;

	while (!pt4) {
		pt4 = ros::topic::waitForMessage<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker_2", ros::Duration(5));
	}

	if (!pt4->markers.empty()) {
		alvarCallback_2(pt4, 1);
	}

	move_group.setPoseTarget(pose);  
	moveit::planning_interface::MoveGroupInterface::Plan my_plan_4;
	bool success_4 = move_group.plan(my_plan_4);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (pose goal) %s", success_4 ? "" : "FAILED");
	if (success_4) {
		move_group.execute(my_plan_4);
		std::cout << "executing..." << std::endl;
	}
	else
		ROS_ERROR("Cannot find a plan.");

	//ROS_INFO_NAMED("tutorial", "Visualizing plan 4 as trajectory line");
	//visual_tools.publishAxisLabeled(pose, "pose4");

	ros::waitForShutdown();
	return 0;
}







