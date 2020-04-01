#include <detector_ros/detector_ros.hpp>

namespace ariitk::detector_ros {

void DetectorROS::init(ros::NodeHandle& nh) {
    img_sub_ = nh.subscribe("/iris/camera_red_iris/image_raw", 1, &DetectorROS::imageCallback, this);

    ros::NodeHandle nh_private("~");

    centre_pub_ = nh_private.advertise<detector_msgs::centre>("centre_coord", 10);
    thresh_pub_= nh_private.advertise<sensor_msgs::Image>("thresh_img", 10);
    contour_pub_ = nh_private.advertise<sensor_msgs::Image>("contours", 10);
    centre_img_pub_ = nh_private.advertise<sensor_msgs::Image>("centre_img", 10);
}

void DetectorROS::run() {
    if(img_.empty()) { return; };

    cv::Mat centre_board = cv::Mat::zeros(img_.size(), CV_8UC3);
    cv::Mat contour_board = cv::Mat::zeros(img_.size(), CV_8UC3);

    detect_.thresholdImage(img_);
    detect_.findGoodContours();
    detect_.drawContours(contour_board);
    detect_.findFrameCentre(centre_board);

    std::pair<int, int> centre_pair = detect_.getCentre();
    centre_coord_.x = centre_pair.first;
    centre_coord_.y = centre_pair.second;
    centre_coord_.header.stamp = ros::Time::now();

    sensor_msgs::ImagePtr thresh_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", detect_.getThresh()).toImageMsg();
    sensor_msgs::ImagePtr contour_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", contour_board).toImageMsg();
    sensor_msgs::ImagePtr centre_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", centre_board).toImageMsg();

    thresh_pub_.publish(thresh_msg);
    contour_pub_.publish(contour_msg);    
    centre_img_pub_.publish(centre_msg);
    centre_pub_.publish(centre_coord_);
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