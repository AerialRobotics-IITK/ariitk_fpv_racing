#include <pose_estimation_ros/pose_estimation_ros.hpp>

namespace ariitk::pose_estimation_ros {


void PoseEstimationROS::init(ros::NodeHandle& nh) {

    centre_coord_sub_ = nh.subscribe("centre_coord", 10, &PoseEstimationROS::centreCallback, this);
    odom_sub_ = nh.subscribe("mavros/local_position/odom", 10, &PoseEstimationROS::odomCallback, this);

    ros::NodeHandle nh_private("~");

    glob_coord_pub_ = nh_private.advertise<detector_msgs::global_coord>("estimated_coord", 10);
    rotation_pub_ = nh_private.advertise<detector_msgs::rotation>("est_rotation", 10);
}

void PoseEstimationROS::run() {
    if((centre_coord_.x == -1) || (centre_coord_.y == -1)) { return; };

    //double dist = centre_coord_.dist;
    pose_est_.getDistance(0.0 /*put dist here*/);
    pose_est_.setImgVec(centre_coord_.x, centre_coord_.y);
    pose_est_.CamToQuad();
    pose_est_.setQuaternion(odom_);
    pose_est_.QuadToGlob(odom_);
    glob_coord_ = pose_est_.getGlobCoord();

    global_coord_.x = glob_coord_(0);
    global_coord_.y = glob_coord_(1);
    global_coord_.z = glob_coord_(2);

    glob_coord_pub_.publish(global_coord_);
}

void PoseEstimationROS::centreCallback(const detector_msgs::centre& msg) {
    centre_coord_ = msg;
}

void PoseEstimationROS::odomCallback(const nav_msgs::Odometry& msg) {
    odom_ = msg;
}

} // ariitk::pose_estimation_ros