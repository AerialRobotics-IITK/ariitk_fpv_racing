#include <fpv_planner/planner.hpp>
#include <iostream>

namespace ariitk::planner {

    void Planner::odom_cb(const nav_msgs::Odometry &msg) {
        odom_ = msg;
    }

    void Planner::TakeOff() {

    }

    void Planner::DetectionBased() {

    }

    void Planner::PrevCoord() {
        //frame_msg = a,b,c

        geometry_msgs::PoseStamped calc_coord; 
        calc_coord.pose.position.x = a;
        calc_coord.pose.position.y = b;
        calc_coord.pose.position.z = c;

        odom_pub_.publish(calc_coord);

        //calculate vector to judge direction so that it doesnt get stuchk in the middle
        //before_pass_vector  - store it   
    }
    
    void Planner::GlobalT() {
        //Rotate drone according to next frame

        //call subscriber to get odom_
        //Find angle b/w old vec and new vec. Convert to quat and rotate

        //Publish rough setpoint until a threshold distance remains between frame and drone
        //After this publish setpoint based on detection

        //AfterPass to BeforePass is the big task .
        //Frame covers 70% area, then we go to BeforePass state

    }
} // namespace ariitk::planner