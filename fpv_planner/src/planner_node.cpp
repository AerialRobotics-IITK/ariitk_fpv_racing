#include <fpv_planner/planner.hpp>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle ph("~");

    ariitk::planner::fsm machine;
    machine.start();

    //auto state = std::async(std::launch::async, ariitk::planner::)

    machine.process_event(ariitk::planner::CmdTakeOff());

    transitRate.sleep();
    machine.process_event(ariitk::planner::CmdEstimated());

    while(//fill condition) {
        transitRate.sleep();
        machine.process_event(ariitk::planner::CmdPass());

        transitRate.sleep();
        machine.process_event(ariitk::planner::CmdGlobalT());
    }
}