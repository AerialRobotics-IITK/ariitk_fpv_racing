#include <planner/planner.hpp>

#define sq(x) (x)*(x)
#define echo(X) std::cout << X << std::endl

namespace msm = boost::msm;
// typedef msm::back::state_machine<fsm> fsm_;
typedef msm::back::state_machine<ariitk::planner::fsm> fsm_;
static char const *const state_names[] = { "Rest", "Hover", "Before", "After" };

namespace ariitk::planner {

    bool fsm::verbose =true;
    void fsm::init(ros::NodeHandle& nh) {
        odom_sub_ = nh.subscribe("mavros/local_position/odom", 10, &fsm::odomCallback, this);
        centre_sub_ = nh.subscribe("centre_coord", 10, &fsm::centreCallback, this);
        est_pose_sub_ = nh.subscribe("estimated_coord", 10, &fsm::estimatedCallback, this);
        state_sub_ = nh.subscribe("mavros/state", 10, &fsm::stateCallback, this);
        front_sub_ = nh.subscribe("front_coord", 10, &fsm::frontCallback, this);

        ROS_WARN("Waiting for state publish"); 
        while (ros::ok() && state_sub_.getNumPublishers()==0) { ros::Duration(0.2).sleep(); }

        arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
        
        pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
        state_pub_ = nh.advertise<std_msgs::String>("curr_state", 10);

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
        pose.pose.position.z = 2.5;

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

            if(odom_.pose.pose.position.z >= 1.8) {
                offb_set_mode.request.custom_mode = "AUTO.LOITER";
                set_mode_client_.call(offb_set_mode);
                ROS_INFO_ONCE("AUTO.LOITER TRIGGERED");
                break;
            }

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
        std::cout << "entering detection based event" << std::endl;
        // mavros_msgs::SetMode offb_set_mode;
        // offb_set_mode.request.custom_mode = "OFFBOARD";
        // set_mode_client_.call(offb_set_mode);
        rough_pose_.x = 4.0;
        rough_pose_.y = 4.0;
        rough_pose_.z = 3.0;

        ros::Rate looprate(20);
        bool flag = true;

        int c = 5;
        float sum_x=0, sum_y=0, sum_z=0;
        while(c>0) {
            
            ros::spinOnce();
            
            std::cout << "summing" << std::endl;
            if(!(centre_.x == -1 || centre_.y ==-1)) {
                sum_x += estimated_pose_.x ;
                sum_y += estimated_pose_.y ;
                sum_z += estimated_pose_.z ;
                c--;
            }
            else {
                std::cout << "i'm fucked, help!" << std::endl;
            }

        }

        x_try = sum_x/5;
        y_try = sum_y/5;
        z_try = sum_z/5;

        // while(flag) {
        //     ros::spinOnce();
        //     // mavros_msgs::SetMode offb_set_mode;
        //     // offb_set_mode.request.custom_mode = "OFFBOARD";
        //     // set_mode_client_.call(offb_set_mode);
        //     // geometry_msgs::PoseStamped pose;
        //     // pose.pose.position.x = 4;
        //     // pose.pose.position.y = 4;
        //     // pose.pose.position.z = 4;
        //     // pose_pub_.publish(pose);
        //     double d = centre_.d;
        //     double x = odom_.pose.pose.position.x;
        //     double y = odom_.pose.pose.position.y;
        //     double z = odom_.pose.pose.position.z;
        //     if(centre_.x == -1 || centre_.y ==-1) {
        //         double dist = sqrt( (sq(rough_pose_.x - x)) +  (sq(rough_pose_.y - y)) + (sq(rough_pose_.z - z)));
        //         if(dist < 2) {
        //             flag = false;
        //             std::cout << "dist = 00 " <<std::endl;
        //             continue;
        //         }
        //         else if (dist >=2 ) {
        //             setpt_.pose.position.x = rough_pose_.x;
        //             setpt_.pose.position.y = rough_pose_.y;
        //             setpt_.pose.position.z = rough_pose_.z;
        //             pose_pub_.publish(setpt_);
        //         }     transition_time
        //     }
        //     else {
        //         double dist = sqrt( (sq(estimated_pose_.x - x)) +  (sq(estimated_pose_.y - y)) + (sq(estimated_pose_.z - z)));
        //         if(dist < 2) {
        //             flag = false;
        //             std::cout << "distance is now <2" << std::endl;
        //             continue;
        //         }
        //         else if (dist >=2 ) {

        //             std::cout << "distance is >2" <<std::endl;
        //             setpt_.pose.position.x = estimated_pose_.x;
        //             setpt_.pose.position.y = estimated_pose_.y;
        //             setpt_.pose.position.z = estimated_pose_.z;
        //             //setpt_.pose.position.z = 2.5;
        //             pose_pub_.publish(setpt_);
        //         }                
        //     }     
        // }

        std::cout << "exitting detection based event" << std::endl;
    }

    void fsm::PrevCoord(CmdPass const &cmd) {
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        set_mode_client_.call(offb_set_mode);
        //frame_msg = a,b,c
        //odom_msg = x,y,z

        //dir_vec = a-x,b-y,c-z;
        //dir_vec = (scalar)*(dir_vec/|dir_vec|);


        // geometry_msgs::PoseStamped calc_coord; [ INFO] [1587244974.432107617, 2526.140000000]: F
        // calc_coord.pose.position.x = x + dir_vec;
        // calc_coord.pose.position.y = y + dir_vec;
        // calc_coord.pose.position.z = z + dir_vec;

        // odom_pub_.publish(calc_coord);

        //calculate vector to judge direction so that it doesnt get stuchk in the middle
        //before_pass_vector  - store it   
        
        std::cout << "we have entered prev coord" << std::endl;
        //trying somehting

        double x_c = odom_.pose.pose.position.x;
        double y_c = odom_.pose.pose.position.y;
        double z_c = odom_.pose.pose.position.z;

        double dist = sqrt( (sq(x_try - x_c)) +  (sq(y_try - y_c)) + (sq(z_try - z_c)));
        int count =0;
        while(dist >=2) {
            ros::spinOnce(); 
            x_c = odom_.pose.pose.position.x;
            y_c = odom_.pose.pose.position.y;
            z_c = odom_.pose.pose.position.z;
            count+=1;
            setpt_.pose.position.x = x_try ;
            setpt_.pose.position.y = y_try ;
            setpt_.pose.position.z = z_try ;
            //std::cout << dist << "is the distance" << std::endl;
            pose_pub_.publish(setpt_);
            dist = sqrt( (sq(x_try - x_c)) +  (sq(y_try - y_c)) + (sq(z_try - z_c)));
        }
        
        // mavros_msgs::SetMode offb_set_mode;
        // offb_set_mode.request.custom_mode = "AUTO.LOITER";
        // set_mode_client_.call(offb_set_mode);
        // ROS_INFO_ONCE("AUTO.LOITER TRIGGERED");

        // double x_c = odom_.pose.pose.position.x;
        // double y_c = odom_.pose.pose.position.y;
        // double z_c = odom_.pose.pose.position.z;

        // double x_e = estimated_pose_.x;
        // double y_e = estimated_pose_.y;
        // double z_e = estimated_pose_.z;
        
       
        // double dist = sqrt( (sq(x_e - x_c)) +  (sq(y_e - y_c)) + (sq(z_e - z_c)));
        // int count =0;
        // while(dist >=0.5) {
        //     ros::spinOnce(); 
        //     x_c = odom_.pose.pose.position.x;
        //     y_c = odom_.pose.pose.position.y;
        //     z_c = odom_.pose.pose.position.z;
        //     x_e = estimated_pose_.x;
        //     y_e = estimated_pose_.y;
        //     z_e = estimated_pose_.z;
        //     count+=1;
        //     setpt_.pose.position.x = estimated_pose_.x;
        //     setpt_.pose.position.y = estimated_pose_.y;
        //     setpt_.pose.position.z = estimated_pose_.z;

        //     pose_pub_.publish(setpt_);
        //     dist = sqrt( (sq(x_e - x_c)) +  (sq(y_e - y_c)) + (sq(z_e - z_c)));
        // }
        // std::cout << "im in the loop for count" << count << std::endl;
         std::cout << "we have exitted from prev coord" << std::endl;
    }
    
    void fsm::GlobalT(CmdGlobalT const &cmd) {

        std::cout<< "entering gloablt event" <<std::endl;

        //trying something

        // rough_pose_.x = 10.0;
        // rough_pose_.y = 3.0;
        // rough_pose_.z = 3.0;

        // while(odom_.pose.pose.position.x <9.5) {
        //     ros::spinOnce();
        //     std::cout << "publishing" <<std::endl;
        //     setpt_.pose.position.x = rough_pose_.x;
        //     setpt_.pose.position.y = rough_pose_.y;
        //     setpt_.pose.position.z = rough_pose_.z;
        //     pose_pub_.publish(setpt_);
        // }        

        //trying something again
        //ros::spinOnce();
        ros::Rate looprate(20);
        double x = odom_.pose.pose.position.x;
        double y = odom_.pose.pose.position.y;
        double z = odom_.pose.pose.position.z;  

        double dist = sqrt( (sq(x_try -x )) +  (sq(y_try -y)) + (sq(z_try -z)) );

        while(dist <2 ) {
            ros::spinOnce();
            looprate.sleep();
            setpt_.pose.position.x = front_pose_.x ;
            setpt_.pose.position.y = front_pose_.y ;
            setpt_.pose.position.z = odom_.pose.pose.position.z;
            pose_pub_.publish(setpt_);
            x = odom_.pose.pose.position.x;
            y = odom_.pose.pose.position.y;
            z = odom_.pose.pose.position.z;
            dist = sqrt( (sq(x_try -x )) +  (sq(y_try -y)) + (sq(z_try -z))) ;
            //std::cout << "distance: " << dist << std::endl;
        }

         mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "AUTO.LOITER";
        set_mode_client_.call(offb_set_mode);
        ROS_INFO_ONCE("AUTO.LOITER TRIGGERED");

        // ros::Rate looprate(20);
        // bool flag = true;
        // while(flag) {
        //     ros::spinOnce();
        //     double x = odom_.pose.pose.position.x;
        //     double y = odom_.pose.pose.position.y;
        //     double z = odom_.pose.pose.position.z;
        //     if(centre_.x == -1 || centre_.y ==-1) {
        //         double dist = sqrt( (sq(rough_pose_.x - x)) +  (sq(rough_pose_.y - y)) + (sq(rough_pose_.z - z)));
        //         if(dist < 2) {
        //             flag = false;
        //             std::cout << "dist = 00  in globalt " <<std::endl;
        //             continue;
        //         }
        //         else if (dist >=2 ) {
        //             setpt_.pose.position.x = rough_pose_.x;
        //             setpt_.pose.position.y = rough_pose_.y;
        //             setpt_.pose.position.z = rough_pose_.z;
        //             pose_pub_.publish(setpt_);
        //         }     
        //     }
        //     else {
        //         double dist = sqrt( (sq(estimated_pose_.x - x)) +  (sq(estimated_pose_.y - y)) + (sq(estimated_pose_.z - z)));
        //         if(dist < 2) {
        //             flag = false;
        //             std::cout << "distance is <2 globalt" <<std::endl;
        //             continue;
        //         }
        //         else if (dist >=2 ) {
        //             std::cout << "distance is >2 globalt" <<std::endl;
        //             setpt_.pose.position.x = estimated_pose_.x;
        //             setpt_.pose.position.y = estimated_pose_.y;
        //             setpt_.pose.position.z = estimated_pose_.z;
        //             pose_pub_.publish(setpt_);
        //         }                
        //     }     
        // }



        int c = 1;
        float sum_x=0, sum_y=0, sum_z=0;
        while(c>0) {

            std::cout << "summing" << std::endl;
            ros::spinOnce();
            sum_x += estimated_pose_.x;
            sum_y += estimated_pose_.y;
            sum_z += estimated_pose_.z;
            c--;
        }

        x_try = sum_x/1;
        y_try = sum_y/1;
        z_try = sum_z/1;

        
        //Rotate drone according to next frame

        //call subscriber to get odom_
        //Find angle b/w old vec and new vec. Convert to quat and rotate

        //Publish rough setpoint until a threshold distance remains between frame and drone
        //After this publish setpoint based on detection

        //AfterPass to BeforePass is the big task .
        //Frame covers 70% area, then we go to BeforePass state

        std::cout << "exitting gloablt state"<<std::endl;

    }

    //helper function
    void fsm::echo_state(fsm_ const &msg) {
        echo("Current state -- " << state_names[msg.current_state()[0]]);
    }
    void fsm::statePublish(ros::NodeHandle nh, fsm_ *fsm) {
        ros::Rate loopRate(10);

        std_msgs::String msg;
        while(ros::ok()) {
            msg.data = state_names[fsm->current_state()[0]];
            state_pub_.publish(msg);
            loopRate.sleep();
        }

    }
} // namespace ariitk::planner