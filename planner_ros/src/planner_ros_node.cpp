#include <planner_ros/planner_ros.hpp>

using namespace ariitk::planner_ros;

int main(int argc,char** argv) {
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;

    fsmROS fsm;

    fsm.init(nh);

    ros::Rate loop_rate(20);

    while(ros::ok()) {
        fsm.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}
