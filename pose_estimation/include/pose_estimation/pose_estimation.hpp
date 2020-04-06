#include <tf/tf.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <detector_msgs/centre>

namespace ariitk::pose_estimation {

class PoseEstimation {
    public :
        PoseEstimation();
        ~PoseEstimation() {};
        void getDistance(float dist);
        void setCamToQaud();
        void setCamMatrix();
        void setImgVec(int x, int y);
        void setQuaternion(nav_msgs::Odometry odom);
        void CamToQuad();
        void QuadToGlob(nav_msgs::Odometry odom);

     private :

        ros::NodeHandle nh("~");

        //ros::Publisher pose_pub_ = nh.advertise<>("estimated_pose",10);

        ros::Subscriber centre_sub_ = nh.Subscriber<detecto>

        Eigen::Matrix3d scale_up_;
        Eigen::Matrix3d cam_matrix_;
        Eigen::Matrix3d cam_to_quad_;
        Eigen::Matrix3d quad_to_glob_;
        Eigen::Quaterniond quat_;
        tf::Quaternion q1_;
        Eigen::Vector3d img_vec_;
        Eigen::Vector3d t_cam_;
        Eigen::Vector3d quad_coord_;
        Eigen::Vector3d glob_coord_;
};

} //pose_estimation

