#include <fpv_planner/planner.hpp>
#include <ros/ros.h>

namespace ariitk::planner_ros {

class fsmROS {
    public:
        fsmROS() {};
        ~fsmROS() {};
        void init(ros::NodeHandle& nh);
        void run();

    private:
        
        ariitk::planner::fsm fsm_;
};

} //namespace ariitk::planner_ros
