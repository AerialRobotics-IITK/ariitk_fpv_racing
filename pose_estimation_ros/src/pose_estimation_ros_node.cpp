#include <pose_estimation_ros/pose_estimation_ros.hpp>

using namespace ariitk::pose_estimation_ros;

int main(int argc,char** argv) {
    ros::init(argc, argv, "pose_estimation_node");
    ros::NodeHandle nh;

    PoseEstimationROS pose_est;

    pose_est.init(nh);

    ros::Rate loop_rate(10);

    while(ros::ok()) {

        loop_rate.sleep();
        ros::spinOnce();
        pose_est.run();
    }

    return 0;

}
