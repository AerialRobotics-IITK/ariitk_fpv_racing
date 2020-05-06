#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <detector_msgs/centre.h>
#include <detector_msgs/global_coord.h>
#include <detector_msgs/rotation.h>
#include <pose_estimation/pose_estimation.hpp>

namespace ariitk::pose_estimation_ros {

class PoseEstimationROS {
	public:
		PoseEstimationROS(){};
		~PoseEstimationROS(){};
		void init(ros::NodeHandle& nh);
		void run();
		void centreCallback(const detector_msgs::centre& msg);
		void odomCallback(const nav_msgs::Odometry& msg);
		void odomdisplay() {
			ROS_INFO_STREAM("x: " << odom_.pose.pose.position.x << "  y: " << odom_.pose.pose.position.y << "  z: " << odom_.pose.pose.position.z << "\n");
		}

	private:
		detector_msgs::centre centre_coord_;
		detector_msgs::global_coord global_coord_;
		detector_msgs::global_coord front_coord_;
		nav_msgs::Odometry odom_;
		Eigen::Vector3d glob_coord_;
		Eigen::Vector3d straight_vec_;

		ros::Subscriber centre_coord_sub_;
		ros::Subscriber odom_sub_;

		ariitk::pose_estimation::PoseEstimation pose_est_;

		ros::Publisher glob_coord_pub_;
		ros::Publisher rotation_pub_;
		ros::Publisher front_coord_pub_;
};

} // namespace ariitk::pose_estimation_ros
