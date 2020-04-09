#include <std_msgs/String.h>
#include <boost/msm/back/mpl_graph_fsm_check.hpp>
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/euml/common.hpp>
#include <boost/msm/front/euml/operator.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/state_machine_def.hpp>

#include <ros/ros.h>
#include <mavros_msgs/SetMode.h> //For TakeOff
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#define echo(X) std::cout << X << std::endl

namespace msm = boost::msm;
namespace mpl = boost::mpl;

namespace ariitk::planner {



//state machine commands

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

class fsm : public msm::front::state_machine_def<fsm>
{
    private :

        nav_msgs::Odometry odom_;

        ros::NodeHandle nh;
        ros::Publisher pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

        ros::Subscriber odom_sub_ = nh.subscribe("mavros/local_position/odom", 10, &fsm::odom_cb,this );
        //Include subscriber for final pose (create msg)



    public:
        bool verbose = true;
        template<class Event,class FSM>
        void on_entry(Event const &, FSM &);
        
        template<class Event,class FSM>
        void on_exit(Event const &, FSM &);

        //state struct definitons

        struct Rest : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                if (verbose){ echo("Entered Rest state"); }
            }

            template<class Event,class FSM>
            void on_exit(Event const &, FSM &)
            {
                if (verbose){ echo("Exited Rest state"); }
            }
        };

        struct Hover : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                if (verbose){ echo("Entered Hover state"); }
            }

            template<class Event,class FSM>
            void on_exit(Event const &, FSM &)
            {
                if (verbose){ echo("Exited Hover state"); }
            }
        };

        struct BeforePass : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                if (verbose){ echo("Entered before_pass state"); }
            }

            template<class Event,class FSM>
            void on_exit(Event const &, FSM &)
            {
                if (verbose){ echo("Exited before_pass state"); }
            }
        };

        struct AfterPass : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                if (verbose){ echo("Entered after_pass state"); }
            }

            template<class Event,class FSM>
            void on_exit(Event const &, FSM &)
            {
                if (verbose){ echo("Exited after_pass state"); }
            }
        };

        //state transition funcitons
        void TakeOff (CmdTakeOff const &cmd);
        void DetectionBased (CmdEstimated const &cmd);
        void PrevCoord (CmdPass const &cmd);
        void GlobalT (CmdGlobalT const &cmd);

        //Callback functions
        void odom_cb(const nav_msgs::Odometry &msg);

        //transition table
        
        struct transition_table : mpl::vector<

        //      Type           Start             Event             Next               Action                      Gaurd 
        // +++ ------- + ----------------- + ---------------- + --------------- + -------------------------- + ----------------------------- +++
        
                a_row<    Rest             ,  CmdTakeOff      ,  Hover          , &fsm::TakeOff                                          >,

        // +++ ------- + ----------------- + ---------------- + --------------- + -------------------------- + ----------------------------- +++

                a_row<    Hover            ,  CmdEstimated    ,  BeforePass     , &fsm::DetectionBased                                   >,

        // +++ ------- + ----------------- + ---------------- + --------------- + -------------------------- + ----------------------------- +++

                a_row<    BeforePass       ,  CmdPass         ,  AfterPass      , &fsm::PrevCoord                                        >,

        // +++ ------- + ----------------- + ---------------- + --------------- + -------------------------- + ----------------------------- +++

                a_row<    AfterPass        ,  CmdGlobalT      ,  BeforePass     , &fsm::GlobalT                                          >

        // +++ ------- + ----------------- + ---------------- + --------------- + -------------------------- + ----------------------------- +++

        >{
        };

};

    
} // namespace ariitk::planner