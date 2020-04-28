#include <planner_ros/planner_ros.hpp>

namespace ariitk::planner_ros {

void fsmROS::init(ros::NodeHandle &nh) {
    
    machine.start();
    ros::NodeHandle ph("~");
    ph.getParam("transition", transition_time);

    machine.init(nh);
    
    machine.process_event(planner::CmdTakeOff());
    machine.process_event(planner::CmdEstimated());
}

void fsmROS::run() {

    ros::Rate transitRate(50);
    
    transitRate.sleep();  
    machine.process_event(planner::CmdPass());

    transitRate.sleep();
    machine.process_event(planner::CmdGlobalT());
    
}

} //namespace ariitk::planner_ros
