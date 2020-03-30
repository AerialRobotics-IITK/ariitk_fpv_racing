#include <pose_estimation/pose_estimation.hpp>

namespace ariitk::pose_estimation {

PoseEstimation::PoseEstimation() {
    
    //set cam _to_quad
    //set cam_matrix_

    img_vec_(0)=0;
    img_vec_(1)=0;
    img_vec_(2)=0;

}
void PoseEstimation::getDistance(float dist=0.0) {
    for(int i = 0; i < 3; i+=1 ){
        for(int j = 0; j<3; j+=1){
            if(i==j)
                scale_up_(i,j) = dist;
            else
                scale_up_(i,j) = 0;
        }
    }
}

void PoseEstimation::setImgVec(int x, int y) {
    img_vec_(0) = x;
    img_vec_(1) = y;
}

void PoseEstimation::setQuaternion(nav_msgs::Odometry odom) {
    q1_ = tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    quat_ = Eigen::Quaterniond(q1_.w(), q1_.x(), q1_.y(), q1_.z());
    quad_to_glob_ = quat_.normalized().toRotationMatrix();
}

void PoseEstimation::CamToQuad() {
    Eigen::Matrix3d inv_cam_matrix = cam_matrix_.inverse();
    quad_coord_ = cam_to_quad_*scale_up_*inv_cam_matrix*img_vec_ + t_cam_;
}

void PoseEstimation::QuadToGlob(nav_msgs::Odometry odom) {
    glob_coord_ = quad_to_glob_*quad_coord_;
    glob_coord_(0) = glob_coord_(0) + odom.pose.pose.position.x;
    glob_coord_(1) = glob_coord_(1) + odom.pose.pose.position.y;
    glob_coord_(2) = glob_coord_(2) + odom.pose.pose.position.z;
        
}

}
