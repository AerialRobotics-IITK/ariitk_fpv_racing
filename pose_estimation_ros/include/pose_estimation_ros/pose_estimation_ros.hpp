#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <detector_msgs/centre.h>
#include <pose_estimation/pose_estimation.hpp>

namespace ariitk::pose_estimation_ros {

class PoseEstimationROS {
    public:
        PoseEstimationROS() {};
        ~PoseEstimationROS() {};
        void init(ros::NodeHandle& nh);
        void run();

    private:
        void centreCallback(const detector_msgs::centre& msg);
        void odomCallback(const nav_msgs::Odometry& msg);

        detector_msgs::centre centre_coord_;
        nav_msgs::Odometry odom_;
        Eigen::Vector3d glob_coord_;

        ros::Subscriber centre_coord_sub_;
        ros::Subscriber odom_sub_;

        ariitk::pose_estimation::PoseEstimation pose_est_;

        //Publishers TBD
        //Publisher Datatypes TBD
};

}// namespace ariitk::pose_estimation_ros

//NOTE: Drone Rotation To Be Done In This Package

