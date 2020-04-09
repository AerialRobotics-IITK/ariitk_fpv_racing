#include <planner/planner.hpp>
#include <iostream>

namespace ariitk::planner {

    void fsm::odom_cb(const nav_msgs::Odometry &msg) {
        odom_ = msg;
    }

    void fsm::TakeOff(CmdTakeOff const &cmd) {

        
    }

    void fsm::DetectionBased(CmdEstimated const &cmd) {

    }

    void fsm::PrevCoord(CmdPass const &cmd) {
        //frame_msg = a,b,c
        //odom_msg = x,y,z

        //dir_vec = a-x,b-y,c-z;
        //dir_vec = (scalar)*(dir_vec/|dir_vec|);


        // geometry_msgs::PoseStamped calc_coord; 
        // calc_coord.pose.position.x = x + dir_vec;
        // calc_coord.pose.position.y = y + dir_vec;
        // calc_coord.pose.position.z = z + dir_vec;

        // odom_pub_.publish(calc_coord);

        //calculate vector to judge direction so that it doesnt get stuchk in the middle
        //before_pass_vector  - store it   
    }
    
    void fsm::GlobalT(CmdGlobalT const &cmd) {
        //Rotate drone according to next frame

        //call subscriber to get odom_
        //Find angle b/w old vec and new vec. Convert to quat and rotate

        //Publish rough setpoint until a threshold distance remains between frame and drone
        //After this publish setpoint based on detection

        //AfterPass to BeforePass is the big task .
        //Frame covers 70% area, then we go to BeforePass state

    }
} // namespace ariitk::planner