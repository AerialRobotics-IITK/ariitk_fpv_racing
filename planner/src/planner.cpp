#include <planner/planner.hpp>

#define sq(x) (x)*(x)
namespace ariitk::planner {

    bool fsm::verbose =true;
    void fsm::init(ros::NodeHandle& nh) {
        odom_sub_ = nh.subscribe("mavros/local_position/odom", 10, &fsm::odomCallback, this);
        centre_sub_ = nh.subscribe("centre_coord", 10, &fsm::centreCallback, this);
        est_pose_sub_ = nh.subscribe("estimated_coord", 10, &fsm::estimatedCallback, this);
        state_sub_ = nh.subscribe("mavros/state", 10, &fsm::stateCallback, this);
       
        ROS_WARN("Waiting for state publish"); 
        while (ros::ok() && state_sub_.getNumPublishers()==0) { ros::Duration(0.2).sleep(); }

        arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
        pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

        ROS_INFO("Initialized Planner Node");
    }

    void fsm::TakeOff(CmdTakeOff const &cmd) {
        ros::Rate rate(20);
        ROS_INFO("Taking off!");

        ROS_WARN("Waiting for connection..");
        while(ros::ok() && current_state_.connected == false) {
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("Connected to MAVROS..");

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 4;

        for(int i = 100; ros::ok() && i > 0; --i){
            pose_pub_.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
        ROS_WARN("Published setpoint 100 times!");
        
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        
        while (!arm_cmd.response.success) {
            arming_client_.call(arm_cmd);
            ROS_INFO_ONCE("Arming...");
        }

        ros::Time last_request = ros::Time::now();

        while(ros::ok()){
            if( current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if( set_mode_client_.call(offb_set_mode)) {
                    if (offb_set_mode.response.mode_sent)
                        ROS_WARN("Offboard enabled");
                }
                last_request = ros::Time::now();
            } 
            else {
                if( !current_state_.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                    if(arming_client_.call(arm_cmd)) {
                        if(arm_cmd.response.success)
                            ROS_WARN("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }
            pose_pub_.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }

    }

    void fsm::DetectionBased(CmdEstimated const &cmd) {
        ros::Rate looprate(20);
        bool flag = true;
        while(flag) {
            ros::spinOnce();
            double x = odom_.pose.pose.position.x;
            double y = odom_.pose.pose.position.y;
            double z = odom_.pose.pose.position.z;
            if(centre_.x == -1 || centre_.y ==-1) {
                double dist = sqrt( (sq(rough_pose_.x - x)) +  (sq(rough_pose_.y - y)) + (sq(rough_pose_.z - z)));
                if(dist < 2) {
                    flag = false;
                    continue;
                }
                else if (dist >=2 ) {
                    setpt_.pose.position.x = rough_pose_.x;
                    setpt_.pose.position.y = rough_pose_.y;
                    setpt_.pose.position.z = rough_pose_.z;
                    pose_pub_.publish(setpt_);
                }     
            }
            else {
                double dist = sqrt( (sq(estimated_pose_.x - x)) +  (sq(estimated_pose_.y - y)) + (sq(estimated_pose_.z - z)));
                if(dist < 2) {
                    flag = false;
                    continue;
                }
                else if (dist >=2 ) {
                    setpt_.pose.position.x = estimated_pose_.x;
                    setpt_.pose.position.y = estimated_pose_.y;
                    setpt_.pose.position.z = estimated_pose_.z;
                    pose_pub_.publish(setpt_);
                }                
            }     
        }
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

        ros::Rate looprate(20);
        bool flag = true;
        while(flag) {
            ros::spinOnce();
            double x = odom_.pose.pose.position.x;
            double y = odom_.pose.pose.position.y;
            double z = odom_.pose.pose.position.z;
            if(centre_.x == -1 || centre_.y ==-1) {
                double dist = sqrt( (sq(rough_pose_.x - x)) +  (sq(rough_pose_.y - y)) + (sq(rough_pose_.z - z)));
                if(dist < 2) {
                    flag = false;
                    continue;
                }
                else if (dist >=2 ) {
                    setpt_.pose.position.x = rough_pose_.x;
                    setpt_.pose.position.y = rough_pose_.y;
                    setpt_.pose.position.z = rough_pose_.z;
                    pose_pub_.publish(setpt_);
                }     
            }
            else {
                double dist = sqrt( (sq(estimated_pose_.x - x)) +  (sq(estimated_pose_.y - y)) + (sq(estimated_pose_.z - z)));
                if(dist < 2) {
                    flag = false;
                    continue;
                }
                else if (dist >=2 ) {
                    setpt_.pose.position.x = estimated_pose_.x;
                    setpt_.pose.position.y = estimated_pose_.y;
                    setpt_.pose.position.z = estimated_pose_.z;
                    pose_pub_.publish(setpt_);
                }                
            }     
        }
        //Rotate drone according to next frame

        //call subscriber to get odom_
        //Find angle b/w old vec and new vec. Convert to quat and rotate

        //Publish rough setpoint until a threshold distance remains between frame and drone
        //After this publish setpoint based on detection

        //AfterPass to BeforePass is the big task .
        //Frame covers 70% area, then we go to BeforePass state

    }
} // namespace ariitk::planner