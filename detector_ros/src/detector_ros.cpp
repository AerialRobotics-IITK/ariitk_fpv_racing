#include <detector_ros/detector_ros.hpp>

namespace ariitk::detector_ros {

void DetectorROS::init(ros::NodeHandle& nh) {
    img_sub_ = nh.subscribe("image_raw", 1, &DetectorROS::imageCallback, this);

    ros::NodeHandle nh_private("~");

    centre_pub_ = nh_private.advertise<detector_msgs::centre>("centre_coord", 10);
    thresh_pub_= nh_private.advertise<sensor_msgs::Image>("thresh_img", 10);
    contour_pub_ = nh_private.advertise<sensor_msgs::Image>("contours", 10);
    centre_img_pub_ = nh_private.advertise<sensor_msgs::Image>("centre_img", 10);
}

void DetectorROS::run() {
    if(img_.empty()) { return; };

    detect_.thresholdImage(img_);
    detect_.findGoodContours();
    detect_.drawContours(img_);
    detect_.findFrameCentre(img_);

    std::pair<int, int> centre_pair = detect_.getCentre();
    centre_coord_.x = centre_pair.first;
    centre_coord_.y = centre_pair.second;
    centre_coord_.header.stamp = ros::Time::now();

    sensor_msgs::ImagePtr thresh_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", detect_.getThresh()).toImageMsg();
    sensor_msgs::ImagePtr contour_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_).toImageMsg();
    sensor_msgs::ImagePtr centre_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_).toImageMsg();

    thresh_pub_.publish(thresh_msg);
    contour_pub_.publish(contour_msg);    
    centre_img_pub_.publish(centre_msg);
    if(centre_coord_.x != -1 && centre_coord_.y != -1) { centre_pub_.publish(centre_coord_); }
}

void DetectorROS::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr_;
    
    try { cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    img_ = cv_ptr_->image;
}

}   // namespace ariitk::detector_ros