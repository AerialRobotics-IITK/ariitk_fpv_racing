#include <planner_ros/planner_ros.hpp>

using namespace ariitk::planner_ros;

int main(int argc,char** argv) {
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);

    fsmROS fsm_ros;
    fsm_ros.init(nh);
    
    while(ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
        fsm_ros.run();
    }

    return 0;

}
