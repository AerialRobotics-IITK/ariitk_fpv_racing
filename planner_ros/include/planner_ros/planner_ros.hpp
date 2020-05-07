#include <planner/planner.hpp>
#include <ros/ros.h>

namespace msm = boost::msm;
namespace mpl = boost::mpl;

typedef msm::back::state_machine<ariitk::planner::FSM> FSM_;

namespace ariitk::planner_ros {

class fsmROS {
	public:
		fsmROS(){};
		~fsmROS(){};
		void init(ros::NodeHandle& nh);
		void run();

	private:
		FSM_ machine;
		float transition_time;
};

} // namespace ariitk::planner_ros
