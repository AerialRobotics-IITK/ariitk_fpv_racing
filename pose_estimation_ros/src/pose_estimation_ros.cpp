#include <pose_estimation_ros/pose_estimation_ros.hpp>

namespace ariitk::pose_estimation_ros {

void PoseEstimationROS::init(ros::NodeHandle& nh) {
	centre_coord_sub_ = nh.subscribe("centre_coord", 10, &PoseEstimationROS::centreCallback, this);
	odom_sub_ = nh.subscribe("mavros/local_position/odom", 10, &PoseEstimationROS::odomCallback, this);

	ros::NodeHandle nh_private("~");

	glob_coord_pub_ = nh_private.advertise<detector_msgs::global_coord>("estimated_coord", 10);
	rotation_pub_ = nh_private.advertise<detector_msgs::rotation>("est_rotation", 10);
	front_coord_pub_ = nh.advertise<detector_msgs::global_coord>("front_coord", 10);
}

void PoseEstimationROS::run() {
	odomdisplay();

	if ((centre_coord_.x == -1) || (centre_coord_.y == -1)) {
		glob_coord_pub_.publish(global_coord_);
		return;
	}
	float dist = 5.0;
	float x_m = 160, y_m = 120;
	pose_est_.getDistance(dist);
	pose_est_.setImgVec(x_m, y_m); // point of the image's centre
	pose_est_.CamToQuad();
	pose_est_.setQuaternion(odom_);
	pose_est_.QuadToGlob(odom_);
	straight_vec_ = pose_est_.getGlobCoord();

	front_coord_.x = straight_vec_(0);
	front_coord_.y = straight_vec_(1);
	front_coord_.z = straight_vec_(2);

	front_coord_pub_.publish(front_coord_);

	dist = centre_coord_.d;
	x_m = centre_coord_.x, y_m = centre_coord_.y;
	pose_est_.getDistance(dist);
	pose_est_.setImgVec(x_m, y_m);
	pose_est_.CamToQuad();
	pose_est_.setQuaternion(odom_);
	pose_est_.QuadToGlob(odom_);
	glob_coord_ = pose_est_.getGlobCoord();

	global_coord_.x = glob_coord_(0);
	global_coord_.y = glob_coord_(1);
	global_coord_.z = glob_coord_(2);

	glob_coord_pub_.publish(global_coord_);
}

void PoseEstimationROS::centreCallback(const detector_msgs::centre& msg) { centre_coord_ = msg; }

void PoseEstimationROS::odomCallback(const nav_msgs::Odometry& msg) { odom_ = msg; }

} // namespace ariitk::pose_estimation_ros