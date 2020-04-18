#include <planner_ros/planner_ros.hpp>

using namespace ariitk::planner_ros;

int main(int argc,char** argv) {
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);

    // for (int i=50;i>0;i--) {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    fsmROS fsm_ros;
    fsm_ros.init(nh);


    while(ros::ok()) {
        //std::cout << "node chal raha hai"<<std::endl;
        //fsm_ros.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}
