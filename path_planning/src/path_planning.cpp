#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <std_srvs/Empty.h>
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
#include <moveit_msgs/PlanningScene.h>
#include <geometric_shapes/shape_operations.h>

geometry_msgs::Pose pose;

bool empty;  // whether the wrist camera can track any ar tags, if not, use the left overhead camera
bool attached = false;  // or publish to a topic

moveit_msgs::CollisionObject addCollisionObject()
{
	sleep(1.0); // To make sure the node can publish  
	moveit_msgs::CollisionObject co;

	shapes::Mesh* m = shapes::createMeshFromResource("package://irb6640_gazebo/models/testbed.dae"); 
	ROS_INFO("mesh loaded");

	shape_msgs::Mesh co_mesh;
	shapes::ShapeMsg co_mesh_msg;  
	shapes::constructMsgFromShape(m, co_mesh_msg);    
	co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg); 

	co.meshes.resize(1);
	co.mesh_poses.resize(1);
	co.meshes[0] = co_mesh;
	co.header.frame_id = "world";
	co.id = "testbed";     

	co.mesh_poses[0].position.x = 4.6;
	co.mesh_poses[0].position.y = 4.6;
	co.mesh_poses[0].position.z = 0.0;
	co.mesh_poses[0].orientation.w= 1.0;
	co.mesh_poses[0].orientation.x= 0.0;
	co.mesh_poses[0].orientation.y= 0.0;
	co.mesh_poses[0].orientation.z= 0.0;   

	co.meshes.push_back(co_mesh);
	co.mesh_poses.push_back(co.mesh_poses[0]);
	co.operation = co.ADD;
	return co;

	//std::vector<moveit_msgs::CollisionObject> vec;
	//vec.push_back(co);
}

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
	tr.setOrigin(tf::Vector3(p.position.x, p.position.y, p.position.z - 0.3));

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
	pose.orientation.y = 0.0;
	pose.orientation.z = 0.707;
	pose.orientation.w = -0.707; 
}

void alvarCallback_3(boost::shared_ptr<ar_track_alvar_msgs::AlvarMarkers const> msg, int marker_number) {   
	ROS_INFO("Move to the target position to pick up the panel...");
	geometry_msgs::Pose p;

	p = msg->markers[marker_number].pose.pose;

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
	tr.setOrigin(tf::Vector3(p.position.x, p.position.y - 0.73, p.position.z-0.02));

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
	pose.orientation.y = 0.0;
	pose.orientation.z = 0.707;
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

	// Define collision objects
	moveit_msgs::CollisionObject collision_object1, collision_object2, collision_object3;
	collision_object1.header.frame_id = move_group.getPlanningFrame();
	collision_object2.header.frame_id = move_group.getPlanningFrame();

	// The id of the object is used to identify it.
	collision_object1.id = "panel1";
	collision_object2.id = "panel2";

	// Define a panel to add to the world.
	shape_msgs::SolidPrimitive primitive1, primitive2;
	primitive1.type = primitive1.BOX;
	primitive1.dimensions.resize(3);
	primitive1.dimensions[0] = 2;
	primitive1.dimensions[1] = 1;
	primitive1.dimensions[2] = 0.05;

	primitive2.type = primitive2.BOX;
	primitive2.dimensions.resize(3);
	primitive2.dimensions[0] = 2;
	primitive2.dimensions[1] = 1;
	primitive2.dimensions[2] = 0.05;

	// Define a pose for the box (specified relative to frame_id)
	geometry_msgs::Pose box_pose1, box_pose2;
	box_pose1.orientation.w = 1.0;
	box_pose1.position.x = 1.5;
	box_pose1.position.y = -1.5;
	box_pose1.position.z = 0.025;

	collision_object1.primitives.push_back(primitive1);
	collision_object1.primitive_poses.push_back(box_pose1);
	collision_object1.operation = collision_object1.ADD;

	box_pose2.orientation.w = 1.0;
	box_pose2.position.x = 3.0;
	box_pose2.position.y = -3.0;
	box_pose2.position.z = 0.025;

	collision_object2.primitives.push_back(primitive2);
	collision_object2.primitive_poses.push_back(box_pose2);
	collision_object2.operation = collision_object2.ADD;

	collision_object3 = addCollisionObject();

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object1);
	//collision_objects.push_back(collision_object2);
	collision_objects.push_back(collision_object3);

	// Now, let's add the collision object into the world
	ROS_INFO_NAMED("tutorial", "Add objects into the world");
	planning_scene_interface.addCollisionObjects(collision_objects);

	// Show text in Rviz of status
	// visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
	visual_tools.trigger();

	// Sleep to allow MoveGroup to recieve and process the collision object message
	ros::Duration(1.0).sleep();
	// Services to turn on/off the vacuum gripper

	ros::ServiceClient client = node_handle.serviceClient<std_srvs::Empty>("irb6640/vacuum_gripper/on");
	ros::ServiceClient client_off = node_handle.serviceClient<std_srvs::Empty>("irb6640/vacuum_gripper/off");
	std_srvs::Empty srv;

	char c;
	std::cout << "input y and press Enter to start planning..." << std::endl;
	std::cin >> c;
	std::cin.ignore(100, '\n');

	/*Plan one*/
	boost::shared_ptr<ar_track_alvar_msgs::AlvarMarkers const> pt;

	// subscribe the topic once and then shutdown
	while (!pt) {
		pt = ros::topic::waitForMessage<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker_3", ros::Duration(5));
	}

	if (!pt->markers.empty()) {
		alvarCallback_3(pt, 0);
	}

	//while (ros::ok()) {
	move_group.setPoseTarget(pose);  
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success = move_group.plan(my_plan);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

	if (success) {
		move_group.execute(my_plan);
		std::cout << "moving to target 1..." << std::endl;
	}
	else
		ROS_ERROR("Cannot find a plan.");

	std::cout << "input y and press Enter to turn on the vacuum gripper..." << std::endl;
	std::cin >> c;
	std::cin.ignore(100, '\n');

	if (client.call(srv)) {
		// Now, let's attach the collision object to the robot.
		ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
		move_group.attachObject(collision_object1.id);

		// Show text in Rviz of status
		//visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
		visual_tools.trigger();

		/* Sleep to allow MoveGroup to recieve and process the attached collision object message */
		ros::Duration(1.0).sleep();

		std::cout << "input y and press Enter to plan next target..." << std::endl;
		std::cin >> c;
		std::cin.ignore(100, '\n');
	}

	/*Plan two*/
	move_group.clearPoseTarget("Manipulator");
	boost::shared_ptr<ar_track_alvar_msgs::AlvarMarkers const> pt2;

	while (!pt2) {
		pt2 = ros::topic::waitForMessage<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker_2", ros::Duration(5));
	}

	if (!pt2->markers.empty()) {
		alvarCallback_2(pt2, 0);
	}

    pose.position.x = 0.929;
	pose.position.y = 1.62;
	pose.position.z = 0.4;

	pose.orientation.x = 0.0;
	pose.orientation.y = 0.0;
	pose.orientation.z = 0.707;
	pose.orientation.w = -0.707; 


	move_group.setPoseTarget(pose);  
	moveit::planning_interface::MoveGroupInterface::Plan my_plan_2;
	bool success_2 = move_group.plan(my_plan_2);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal) %s", success_2 ? "" : "FAILED");

	if (success_2) {
		move_group.execute(my_plan_2);
		std::cout << "moving to target 2..." << std::endl;
	}
	else
		ROS_ERROR("Cannot find a plan.");

	std::cout << "input y and press Enter to plan next target..." << std::endl;
	std::cin >> c;
	std::cin.ignore(100, '\n');

	pose.position.z = pose.position.z - 0.15;
	move_group.setPoseTarget(pose);  
	moveit::planning_interface::MoveGroupInterface::Plan my_plan_2_2;
	bool success_2_2 = move_group.plan(my_plan_2_2);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 2_2 (pose goal) %s", success_2_2 ? "" : "FAILED");

	if (success_2_2) {
		move_group.execute(my_plan_2_2);
		std::cout << "moving to target 2_2..." << std::endl;
	}
	else
		ROS_ERROR("Cannot find a plan.");

	std::cout << "input y and press Enter to turn off vacuum gripper..." << std::endl;
	std::cin >> c;
	std::cin.ignore(100, '\n');

	if (client_off.call(srv)) {
		// Now, let's detach the collision object from the robot.
		ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
		move_group.detachObject(collision_object1.id);

		// Show text in Rviz of status
		//visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
		visual_tools.trigger();

		/* Sleep to allow MoveGroup to recieve and process the detach collision object message */
		ros::Duration(1.0).sleep();

		std::cout << "input y and press Enter to plan next target..." << std::endl;
		std::cin >> c;
		std::cin.ignore(100, '\n');
	} 

	/*Plan three*/
	move_group.clearPoseTarget("Manipulator");
	boost::shared_ptr<ar_track_alvar_msgs::AlvarMarkers const> pt3;

	while (!pt3) {
		pt3 = ros::topic::waitForMessage<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker_3", ros::Duration(5));
	}

	if (!pt3->markers.empty()) {
		alvarCallback_3(pt3, 0);
	}

	move_group.setPoseTarget(pose);  
	moveit::planning_interface::MoveGroupInterface::Plan my_plan_3;
	bool success_3 = move_group.plan(my_plan_3);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (pose goal) %s", success_3 ? "" : "FAILED");

	if (success_3) {
		move_group.execute(my_plan_3);
		std::cout << "moving to target 3..." << std::endl;
	}
	else
		ROS_ERROR("Cannot find a plan.");

	std::cout << "input y and press Enter to turn on the vacuum gripper..." << std::endl;
	std::cin >> c;
	std::cin.ignore(100, '\n');

	if (client.call(srv)) {
		//  Now, let's attach the collision object to the robot.
		ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
		move_group.attachObject(collision_object2.id);

		// Show text in Rviz of status
		//visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
		visual_tools.trigger();

		/* Sleep to allow MoveGroup to recieve and process the attached collision object message */
		ros::Duration(1.0).sleep();
		std::cout << "input y and press Enter to plan next target..." << std::endl;
		std::cin >> c;
		std::cin.ignore(100, '\n');
	}

	/*Plan four*/
	move_group.clearPoseTarget("Manipulator");
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
		std::cout << "moving to target 4..." << std::endl;
	}
	else
		ROS_ERROR("Cannot find a plan.");

	std::cout << "input y and press Enter to plan next target..." << std::endl;
	std::cin >> c;
	std::cin.ignore(100, '\n');

	pose.position.z = pose.position.z - 0.2;
	move_group.setPoseTarget(pose);  
	moveit::planning_interface::MoveGroupInterface::Plan my_plan_4_2;
	bool success_4_2 = move_group.plan(my_plan_4_2);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 4_2 (pose goal) %s", success_4_2 ? "" : "FAILED");

	if (success_4_2) {
		move_group.execute(my_plan_4_2);
		std::cout << "moving to target 4_2..." << std::endl;
	}
	else
		ROS_ERROR("Cannot find a plan.");

	std::cout << "input y and press Enter to turn off vacuum gripper..." << std::endl;
	std::cin >> c;
	std::cin.ignore(100, '\n');

	if (client_off.call(srv)) {
		// Now, let's detach the collision object from the robot.
		ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
		move_group.detachObject(collision_object2.id);

		// Show text in Rviz of status
		//visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
		visual_tools.trigger();

		/* Sleep to allow MoveGroup to recieve and process the detach collision object message */
		ros::Duration(1.0).sleep();
		std::cout << "input y and press Enter to move to next target..." << std::endl;
		std::cin >> c;
		std::cin.ignore(100, '\n');
	} 

	ros::waitForShutdown();
	return 0;
}


