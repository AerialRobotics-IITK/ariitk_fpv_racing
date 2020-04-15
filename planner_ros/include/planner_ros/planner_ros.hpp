#include <planner/planner.hpp>
#include <ros/ros.h>

namespace msm = boost::msm;
namespace mpl = boost::mpl;

typedef msm::back::state_machine<ariitk::planner::fsm> fsm_;

namespace ariitk::planner_ros {

class fsmROS {
    public:
        fsmROS() {};
        ~fsmROS() {};
        void init(ros::NodeHandle& nh);
        void run();

    private:
        fsm_ machine;

        //ariitk::planner::fsm fsm_p;

};

} //namespace ariitk::planner_ros
