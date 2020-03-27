#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <detector_msgs/centre.h>
#include <detector/detector.hpp>
#include <image_transport/image_transport.h>

namespace ariitk::detector_ros{
    class DetectorROS {
        public:
            DetectorROS() {};
            ~DetectorROS() {};
            void init(ros::NodeHandle& nh);
            void run();

        private:
            void imageCallback(const sensor_msgs::ImageConstPtr& msg);
            cv::Mat img_;
            ros::Subscriber img_sub_;
            ros::Publisher centre_pub_;
            ros::Publisher topic_pub_;
            
            ariitk::detector::Detector detect_;
            detector_msgs::centre centre_coord_;

    };
}