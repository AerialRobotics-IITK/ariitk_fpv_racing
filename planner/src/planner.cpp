#include <planner/planner.hpp>

#define sq(x) (x) * (x)
#define echo(X) std::cout << X << std::endl
int p = 0;
double yaw_change = 0;

namespace msm = boost::msm;
typedef msm::back::state_machine<ariitk::planner::fsm> fsm_;
static char const* const state_names[] = {"Rest", "Hover", "Before", "After"};

namespace ariitk::planner {

bool fsm::verbose = true;
void fsm::init(ros::NodeHandle& nh) {
	rough_pose_[0][0] = 5.0;
	rough_pose_[0][1] = 0.0;
	rough_pose_[0][2] = 2.7;
	rough_pose_[1][0] = 8.0;
	rough_pose_[1][1] = -6.0;
	rough_pose_[1][2] = 2.7;
	rough_pose_[2][0] = 4.0;
	rough_pose_[2][1] = -9.0;
	rough_pose_[2][2] = 2.7;
	rough_pose_[3][0] = 2.0;
	rough_pose_[3][1] = -4.0;
	rough_pose_[3][2] = 2.7;

	odom_sub_ = nh.subscribe("mavros/local_position/odom", 10, &fsm::odomCallback, this);
	centre_sub_ = nh.subscribe("centre_coord", 10, &fsm::centreCallback, this);
	est_pose_sub_ = nh.subscribe("estimated_coord", 10, &fsm::estimatedCallback, this);
	state_sub_ = nh.subscribe("mavros/state", 10, &fsm::stateCallback, this);
	front_sub_ = nh.subscribe("front_coord", 10, &fsm::frontCallback, this);

	ROS_WARN("Waiting for state publish");
	while (ros::ok() && state_sub_.getNumPublishers() == 0) { ros::Duration(0.2).sleep(); }

	arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	state_pub_ = nh.advertise<std_msgs::String>("curr_state", 10);

	ROS_INFO("Initialized Planner Node");
}

void fsm::TakeOff(CmdTakeOff const& cmd) {
	ros::Rate rate(20);
	ROS_INFO("Taking off!");

	ROS_WARN("Waiting for connection..");
	while (ros::ok() && current_state_.connected == false) {
		ros::spinOnce();
		rate.sleep();
	}
	ROS_INFO("Connected to MAVROS..");

	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = 0;
	pose.pose.position.y = 0;
	pose.pose.position.z = 2.5;

	for (int i = 100; ros::ok() && i > 0; --i) {
		pose_pub_.publish(pose);
		ros::spinOnce();
		rate.sleep();
	}
	ROS_WARN("Published setpoint 100 times!");

	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";

	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	while (!arm_cmd.response.success) {
		arming_client_.call(arm_cmd);
		ROS_INFO_ONCE("Arming...");
	}

	ros::Time last_request = ros::Time::now();

	while (ros::ok()) {
		if (odom_.pose.pose.position.z >= 1.8) {
			offb_set_mode.request.custom_mode = "AUTO.LOITER";
			set_mode_client_.call(offb_set_mode);
			ROS_INFO_ONCE("AUTO.LOITER Triggered");
			break;
		}

		if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
			if (set_mode_client_.call(offb_set_mode)) {
				if (offb_set_mode.response.mode_sent) ROS_WARN("Offboard enabled");
			}
			last_request = ros::Time::now();
		} else {
			if (!current_state_.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
				if (arming_client_.call(arm_cmd)) {
					if (arm_cmd.response.success) ROS_WARN("Vehicle armed");
				}
				last_request = ros::Time::now();
			}
		}
		pose_pub_.publish(pose);
		ros::spinOnce();
		rate.sleep();
	}
}

void fsm::DetectionBased(CmdEstimated const& cmd) {
	ROS_INFO("Entering detection based event");

	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";
	set_mode_client_.call(offb_set_mode);

	ros::Rate looprate(20);
	bool flag = true;
	float sum_x = 0, sum_y = 0, sum_z = 0;
	int k = 1;

	while (flag) {
		ros::spinOnce();

		if (!(centre_.x == -1 || centre_.y == -1) && k == 1) {
			int c = 5;
			while (c > 0) {
				ros::spinOnce();
				if (!(centre_.x == -1 || centre_.y == -1)) {
					sum_x += estimated_pose_.x;
					sum_y += estimated_pose_.y;
					sum_z += estimated_pose_.z;
					c--;
				} else {
					std::cout << "centre is not being detected" << std::endl;
				}
			}

			frame_vec_(0) = sum_x / 5;
			frame_vec_(1) = sum_y / 5;
			frame_vec_(2) = sum_z / 5;
			std::cout << frame_vec_(0) << " " << frame_vec_(1) << " " << frame_vec_(2) << std::endl;
			k--;
		}

		double d = centre_.d;
		double x = odom_.pose.pose.position.x;
		double y = odom_.pose.pose.position.y;
		double z = odom_.pose.pose.position.z;

		if ((centre_.x == -1 || centre_.y == -1) && (sum_x == 0 && sum_y == 0 && sum_z == 0)) {
			double dist = sqrt((sq(rough_pose_[p % 4][0] - x)) + (sq(rough_pose_[p % 4][1] - y)));
			if (dist < 2) {
				flag = false;
				continue;
			} else if (dist >= 2) {
				setpt_.pose.position.x = rough_pose_[p % 4][0];
				setpt_.pose.position.y = rough_pose_[p % 4][1];
				setpt_.pose.position.z = rough_pose_[p % 4][2];
				pose_pub_.publish(setpt_);
			}
		} else {
			double dist = sqrt((sq(estimated_pose_.x - x)) + (sq(estimated_pose_.y - y)) + (sq(estimated_pose_.z - z)));
			if (dist < 2) {
				flag = false;
				continue;
			} else if (dist >= 2) {
				setpt_.pose.position.x = frame_vec_(0);
				setpt_.pose.position.y = frame_vec_(1);
				setpt_.pose.position.z = frame_vec_(2);
				pose_pub_.publish(setpt_);
			}
		}
	}

	ROS_INFO("Exitting detection based event");
}

void fsm::PrevCoord(CmdPass const& cmd) {
	ros::spinOnce();
	ROS_INFO("Entering PrevCoord event");

	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";
	set_mode_client_.call(offb_set_mode);

	ros::Rate lprt(30);            // THIS LOOP IS ONLY FOR STALLING
	for (int i = 0; i < 20; i++) { // MAY BE REMOVED LATER
		setpt_.pose.position.x = odom_.pose.pose.position.x;
		setpt_.pose.position.y = odom_.pose.pose.position.y;
		setpt_.pose.position.z = odom_.pose.pose.position.z;
		pose_pub_.publish(setpt_);

		lprt.sleep();
	}

	ros::spinOnce();
	drone_vec_(0) = odom_.pose.pose.position.x;
	drone_vec_(1) = odom_.pose.pose.position.y;
	drone_vec_(2) = odom_.pose.pose.position.z;
	double dist = sqrt((sq(frame_vec_(0) - drone_vec_(0))) + (sq(frame_vec_(1) - drone_vec_(1))) + (sq(frame_vec_(2) - drone_vec_(2))));

	traj_vec_(0) = ((frame_vec_(0) - drone_vec_(0)) / dist) * 2;
	traj_vec_(1) = ((frame_vec_(1) - drone_vec_(1)) / dist) * 2;
	traj_vec_(2) = ((frame_vec_(2) - drone_vec_(2)) / dist) * 2;

	while (dist < 2.0) {
		ros::spinOnce();
		drone_vec_(0) = odom_.pose.pose.position.x;
		drone_vec_(1) = odom_.pose.pose.position.y;
		drone_vec_(2) = odom_.pose.pose.position.z;

		setpt_.pose.position.x = drone_vec_(0) + traj_vec_(0);
		setpt_.pose.position.y = drone_vec_(1) + traj_vec_(1);
		setpt_.pose.position.z = 2.7 /* drone_vec_(2) + traj_vec_(2) */;

		pose_pub_.publish(setpt_);
		dist = sqrt((sq(frame_vec_(0) - drone_vec_(0))) + (sq(frame_vec_(1) - drone_vec_(1))) + (sq(frame_vec_(2) - drone_vec_(2))));
	}

	double v1x, v1y, v2x, v2y;
	v1x = front_pose_.x - odom_.pose.pose.position.x;
	v1y = front_pose_.y - odom_.pose.pose.position.y;
	double mod_v1 = sqrt(sq(v1x) + sq(v1y));
	v2x = rough_pose_[(p % 4) + 1][0] - odom_.pose.pose.position.x;
	v2y = rough_pose_[(p % 4) + 1][1] - odom_.pose.pose.position.y;
	double mod_v2 = sqrt(sq(v2x) + sq(v2y));
	double crossp = ((v1x * v2y) - (v1y * v2x)) / (mod_v1 * mod_v2);

	yaw_change += asin(crossp);
	std::cout << crossp << std::endl << v1x << " " << v1y << " " << v2x << " " << v2y << std::endl << mod_v1 << " " << mod_v2 << std::endl;

	for (int i = 0; i < 40; i++) {
		setpt_.pose.position.x = odom_.pose.pose.position.x;
		setpt_.pose.position.y = odom_.pose.pose.position.y;
		setpt_.pose.position.z = odom_.pose.pose.position.z;
		setpt_.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_change);
		pose_pub_.publish(setpt_);

		lprt.sleep();
	}

	p++;
	ROS_INFO("Exiting PrevCoord event");
}

void fsm::GlobalT(CmdGlobalT const& cmd) {
	ROS_INFO("Entering GlobalT event");

	bool flag = true;
	int k = 1;

	while (flag) {
		ros::spinOnce();

		float sum_x = 0, sum_y = 0, sum_z = 0;
		if (!(centre_.x == -1 || centre_.y == -1) && k == 1) {
			int c = 1;
			ros::spinOnce();
			ros::Rate lprt(30);             // THIS LOOP IS ONLY FOR STALLING
			for (int i = 0; i < 100; i++) { // MAY BE REMOVED LATER
				setpt_.pose.position.x = odom_.pose.pose.position.x;
				setpt_.pose.position.y = odom_.pose.pose.position.y;
				setpt_.pose.position.z = odom_.pose.pose.position.z;
				pose_pub_.publish(setpt_);

				lprt.sleep();
			}
			while (c > 0) {
				if (!(centre_.x == -1 || centre_.y == -1)) {
					std::cout << "summing" << std::endl;
					sum_x += estimated_pose_.x;
					sum_y += estimated_pose_.y;
					sum_z += estimated_pose_.z;
					c--;
				} else {
					std::cout << "center is not being detected" << std::endl;
				}
			}

			frame_vec_(0) = sum_x / 1;
			frame_vec_(1) = sum_y / 1;
			frame_vec_(2) = sum_z / 1;
			std::cout << frame_vec_(0) << " " << frame_vec_(1) << " " << frame_vec_(2) << std::endl;
			if (frame_vec_(2) > 0) k--;
		}

		double d = centre_.d;
		double x = odom_.pose.pose.position.x;
		double y = odom_.pose.pose.position.y;
		double z = odom_.pose.pose.position.z;

		if ((centre_.x == -1 || centre_.y == -1) && k) {
			double dist = sqrt((sq(rough_pose_[p % 4][0] - x)) + (sq(rough_pose_[p % 4][1] - y)));
			if (dist < 2) {
				flag = false;
				continue;
			} else if (dist >= 2) {
				setpt_.pose.position.x = rough_pose_[p % 4][0];
				setpt_.pose.position.y = rough_pose_[p % 4][1];
				setpt_.pose.position.z = rough_pose_[p % 4][2];
				pose_pub_.publish(setpt_);
			}
		} else {
			double dist = sqrt((sq(estimated_pose_.x - x)) + (sq(estimated_pose_.y - y)) + (sq(estimated_pose_.z - z)));
			if (dist < 2) {
				flag = false;
				continue;
			} else if (dist >= 2) {
				setpt_.pose.position.x = frame_vec_(0);
				setpt_.pose.position.y = frame_vec_(1);
				setpt_.pose.position.z = 2.7 /* frame_vec_(2) */;
				pose_pub_.publish(setpt_);
			}
		}
	}

	ROS_INFO("Exiting GlobalT event");
}

// helper function
void fsm::echo_state(fsm_ const& msg) { echo("Current state -- " << state_names[msg.current_state()[0]]); }
void fsm::statePublish(ros::NodeHandle nh, fsm_* fsm) {
	ros::Rate loopRate(10);

	std_msgs::String msg;
	while (ros::ok()) {
		msg.data = state_names[fsm->current_state()[0]];
		state_pub_.publish(msg);
		loopRate.sleep();
	}
}
} // namespace ariitk::planner