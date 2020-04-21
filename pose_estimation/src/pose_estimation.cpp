#include <pose_estimation/pose_estimation.hpp>

namespace ariitk::pose_estimation {

PoseEstimation::PoseEstimation() {
    
    //set cam _to_quad_
    //set cam_matrix_

    // cam_matrix_(0,0) = 277.191356;
    // cam_matrix_(0,1) = 0;
    // cam_matrix_(0,2) = 320.5;
    // cam_matrix_(1,0) = 0;
    // cam_matrix_(1,1) = 277.191356;
    // cam_matrix_(1,2) = 240.5;
    // cam_matrix_(2,0) = 0;
    // cam_matrix_(2,1) = 0;
    // cam_matrix_(2,2) = 1.0;

    cam_matrix_(0,0) = 277.191356;
    cam_matrix_(0,1) = 0;
    cam_matrix_(0,2) = 160.5;
    cam_matrix_(1,0) = 0;
    cam_matrix_(1,1) = 277.191356;
    cam_matrix_(1,2) = 120.5;
    cam_matrix_(2,0) = 0;
    cam_matrix_(2,1) = 0;
    cam_matrix_(2,2) = 1.0;

    // cam_matrix_(0,0) = 476.7030836014194;
    // cam_matrix_(0,1) = 0;
    // cam_matrix_(0,2) = 400.5;
    // cam_matrix_(1,0) = 0;
    // cam_matrix_(1,1) = 476.7030836014194;
    // cam_matrix_(1,2) = 400.5;
    // cam_matrix_(2,0) = 0;
    // cam_matrix_(2,1) = 0;
    // cam_matrix_(2,2) = 1;
    
    cam_to_quad_(0,0) = 0;
    cam_to_quad_(0,1) = 0;
    cam_to_quad_(0,2) = 1;
    cam_to_quad_(1,0) = -1;
    cam_to_quad_(1,1) = 0;
    cam_to_quad_(1,2) = 0;
    cam_to_quad_(2,0) = 0;
    cam_to_quad_(2,1) = -1;
    cam_to_quad_(2,2) = 0;


    img_vec_(0)=0;
    img_vec_(1)=0;
    img_vec_(2)=1;

    t_cam_(0)=0.0;
    t_cam_(1)=0.5;
    t_cam_(2)=0.02;
    
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

void PoseEstimation::setImgVec(float x, float y) {
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
    std::cout << scale_up_(0,0) <<std::endl;
    Eigen::Matrix3d inv_cam_matrix = cam_matrix_.inverse();
    quad_coord_ = cam_to_quad_*scale_up_*inv_cam_matrix*img_vec_ + t_cam_;
    //quad_coord_ = inv_cam_matrix*img_vec_;

    //std::cout << quad_coord_ <<std::endl;
    }

void PoseEstimation::QuadToGlob(nav_msgs::Odometry odom) {
    glob_coord_ = quad_to_glob_*quad_coord_;
    //std::cout << odom.pose.pose.position.x << std::endl;
    glob_coord_(0) = glob_coord_(0) + odom.pose.pose.position.x;
    glob_coord_(1) = glob_coord_(1) + odom.pose.pose.position.y;
    glob_coord_(2) = glob_coord_(2) + odom.pose.pose.position.z;
    //glob_coord_(1) = -(glob_coord_(1)+glob_coord_(2));
    
        
}

} // ariitk::pose_estimation
