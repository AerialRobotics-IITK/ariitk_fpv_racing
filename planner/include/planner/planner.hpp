#include <boost/msm/back/mpl_graph_fsm_check.hpp>
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/euml/common.hpp>
#include <boost/msm/front/euml/operator.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <iostream>
#include <math.h>
#include <std_msgs/String.h>
#include <vector>

#include <detector_msgs/centre.h>
#include <detector_msgs/global_coord.h>
#include <detector_msgs/rotation.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/tf.h>

#define echo(X) std::cout << X << std::endl

namespace msm = boost::msm;
namespace mpl = boost::mpl;

namespace ariitk::planner {

// state machine commands

struct CmdTakeOff {
	CmdTakeOff() {}
};

struct CmdEstimated {
	CmdEstimated() {}
};

struct CmdPass {
	CmdPass() {}
};

struct CmdGlobalT {
	CmdGlobalT() {}
};

class FSM : public msm::front::state_machine_def<FSM> {
	typedef msm::active_state_switch_after_transition_action active_state_switch_policy;

	protected:
		nav_msgs::Odometry odom_;
		detector_msgs::centre centre_;
		detector_msgs::global_coord estimated_pose_;
		detector_msgs::global_coord front_pose_;
		geometry_msgs::PoseStamped setpt_;
		mavros_msgs::State current_state_;
		Eigen::Vector3d frame_vec_;
		Eigen::Vector3d drone_vec_;
		Eigen::Vector3d traj_vec_;
		double rough_pose_[4][3];
		double thresh_dist_;
		double yaw_change_;
		int p_;

		ros::Publisher pose_pub_;
		ros::Publisher state_pub_;

		ros::Subscriber odom_sub_;
		ros::Subscriber centre_sub_;
		ros::Subscriber est_pose_sub_;
		ros::Subscriber state_sub_;
		ros::Subscriber front_sub_;

		ros::ServiceClient arming_client_;
		ros::ServiceClient set_mode_client_;

	public:
		static bool verbose;
		template<class Event, class FSM> void on_entry(Event const&, FSM&){};

		template<class Event, class FSM> void on_exit(Event const&, FSM&){};

		// state struct definitons

		struct Rest : public msm::front::state<> {
			template<class Event, class FSM> void on_entry(Event const&, FSM&) {
				if (verbose) { echo("Entered Rest state"); }
			}

			template<class Event, class FSM> void on_exit(Event const&, FSM&) {
				if (verbose) { echo("Exited Rest state"); }
			}
		};

		struct Hover : public msm::front::state<> {
			template<class Event, class FSM> void on_entry(Event const&, FSM&) {
				if (verbose) { echo("Entered Hover state"); }
			}

			template<class Event, class FSM> void on_exit(Event const&, FSM&) {
				if (verbose) { echo("Exited Hover state"); }
			}
		};

		struct BeforePass : public msm::front::state<> {
			template<class Event, class FSM> void on_entry(Event const&, FSM&) {
				if (verbose) { echo("Entered before_pass state"); }
			}

			template<class Event, class FSM> void on_exit(Event const&, FSM&) {
				if (verbose) { echo("Exited before_pass state"); }
			}
		};

		struct AfterPass : public msm::front::state<> {
			template<class Event, class FSM> void on_entry(Event const&, FSM&) {
				if (verbose) { echo("Entered after_pass state"); }
			}

			template<class Event, class FSM> void on_exit(Event const&, FSM&) {
				if (verbose) { echo("Exited after_pass state"); }
			}
		};

		typedef Rest initial_state;
		typedef msm::back::state_machine<FSM> FSM_;

		void init(ros::NodeHandle& nh);

		// state transition funcitons
		void TakeOff(CmdTakeOff const& cmd);
		void DetectionBased(CmdEstimated const& cmd);
		void PrevCoord(CmdPass const& cmd);
		void GlobalT(CmdGlobalT const& cmd);
		void echo_state(FSM_ const& msg);
		void statePublish(ros::NodeHandle nh, FSM_* FSM);

		// Callback functions
		void frontCallback(const detector_msgs::global_coord& msg) { front_pose_ = msg; };
		void odomCallback(const nav_msgs::Odometry& msg) { odom_ = msg; };
		void centreCallback(const detector_msgs::centre& msg) { centre_ = msg; };
		void estimatedCallback(const detector_msgs::global_coord& msg) { estimated_pose_ = msg; };
		void stateCallback(const mavros_msgs::State& msg) { current_state_ = msg; };

		// transition table

		struct transition_table
			: mpl::vector<

				//      Type           Start             Event             Next               Action                      Gaurd
				// +++ ------- + ----------------- + ---------------- + --------------- + -------------------------- + ----------------------------- +++

				a_row<Rest, CmdTakeOff, Hover, &FSM::TakeOff>,

				// +++ ------- + ----------------- + ---------------- + --------------- + -------------------------- + ----------------------------- +++

				a_row<Hover, CmdEstimated, BeforePass, &FSM::DetectionBased>,

				// +++ ------- + ----------------- + ---------------- + --------------- + -------------------------- + ----------------------------- +++

				a_row<BeforePass, CmdPass, AfterPass, &FSM::PrevCoord>,

				// +++ ------- + ----------------- + ---------------- + --------------- + -------------------------- + ----------------------------- +++

				a_row<AfterPass, CmdGlobalT, BeforePass, &FSM::GlobalT>

				// +++ ------- + ----------------- + ---------------- + --------------- + -------------------------- + ----------------------------- +++

				> {};
};

} // namespace ariitk::planner