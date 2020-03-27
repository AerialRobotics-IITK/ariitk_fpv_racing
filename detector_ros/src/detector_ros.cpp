#include <detector_ros/detector_ros.hpp>

namespace ariitk::detector_ros {

void DetectorROS::init(ros::NodeHandle& nh) {
    ROS_INFO_STREAM("test_init");
    centre_pub_ = nh.advertise<detector_msgs::centre>("centre_coord",10);
    img_sub_ =nh.subscribe("/iris/camera_red_iris/image_raw",1, &DetectorROS::imageCallback, this);
    topic_pub_= nh.advertise<sensor_msgs::Image>("testing",10);
}

void DetectorROS::run() {

    detect_.thresholdImage(img_);
    cv::Mat img = detect_.returnThresh();
    cv::Mat test = cv::Mat::zeros(img.size(), CV_8UC3);

    detect_.findGoodContours();

    detect_.findFrameCentre(test);

    //detect_.drawContours(test);

    std::pair<int, int> centre_pair = detect_.getCentre();
    centre_coord_.x = centre_pair.first;
    centre_coord_.y = centre_pair.second;
    centre_coord_.header.stamp = ros::Time::now();

    sensor_msgs::ImagePtr thresh_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", test).toImageMsg();
    topic_pub_.publish(thresh_img);
    test = cv::Mat::zeros(img.size(), CV_8UC3);
    
    centre_pub_.publish(centre_coord_);
}

void DetectorROS::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr_;
    try
    {
       cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    img_ = cv_ptr_->image;
}
}