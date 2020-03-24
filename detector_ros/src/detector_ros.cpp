#include <detector_ros/detector_ros.hpp>

namespace ariitk::detector_ros {

void DetectorROS::init(ros::NodeHandle& nh) {
    centre_pub_ = nh.advertise<detector_msgs::centre>("centre_coord",1);
    img_sub_ =nh.subscribe("/iris/camera_red_iris/image_raw",1, &DetectorROS::imageCallback, this);

}

void DetectorROS::run() {

    //cv::Mat img = img_ptr_->image;
    detect_.thresholdImage(img_ptr_->image);

    detect_.findGoodContours();

    detect_.findFrameCentre();

    
    std::pair<int, int> centre_pair = detect_.getCentre();
    centre_coord_.x = centre_pair.first;
    centre_coord_.y = centre_pair.second;
    centre_coord_.header.stamp = ros::Time::now();

    centre_pub_.publish(centre_coord_);
}

void DetectorROS::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try
    {
       img_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
}
}