#include <planner_ros/planner_ros.hpp>

namespace ariitk::planner_ros {

void fsmROS::init(ros::NodeHandle &nh) {
    
    machine.start();
    ros::NodeHandle ph("~");
    ph.getParam("transition", transition_time);
    // ariitk::planner::fsm fsm_;

    // fsm_.init(ph);
    machine.init(nh);
    
    //auto state = std::async(std::launch::async, planner::fsm::statePublish, ph, &machine);
    machine.process_event(planner::CmdTakeOff());
}

void fsmROS::run() {

    // std::cout<<"start ho gaya"<<std::endl;
    ros::Rate transitRate(1/transition_time);
    machine.process_event(planner::CmdEstimated());

    transitRate.sleep();  //have to write in param file
    machine.process_event(planner::CmdPass());

    transitRate.sleep();
    machine.process_event(planner::CmdGlobalT());
    
}

} //namespace ariitk::planner_ros
