#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <detector/detector.hpp>
#include <detector_msgs/centre.h>

namespace ariitk::detector_ros {

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
        ros::Publisher thresh_pub_;
        ros::Publisher contour_pub_;
        ros::Publisher centre_img_pub_;
        
        ariitk::detector::Detector detect_;
        
        detector_msgs::centre centre_coord_;
};

} // namespace ariitk::detector_ros
