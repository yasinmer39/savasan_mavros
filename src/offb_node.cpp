#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamValue.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mavros_msgs/CommandLong.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){

    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("uav0/mavros/state", 10, state_cb);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("uav0/mavros/setpoint_velocity/cmd_vel", 10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav0/mavros/setpoint_position/local", 10);
    ros::Publisher attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("uav0/mavros/setpoint_raw/attitude", 10);

    // Create a service client to send MAVROS commands
    ros::ServiceClient takeoffClient = nh.serviceClient<mavros_msgs::CommandTOL>
            ("uav0/mavros/cmd/takeoff");
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav0/mavros/set_mode");
    ros::ServiceClient set_param_vel = nh.serviceClient<mavros_msgs::ParamSet>
            ("uav0/mavros/param/set");
    ros::ServiceClient cmd_client = nh.serviceClient<mavros_msgs::CommandLong>
            ("uav0/mavros/cmd/command");
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
    	std::cout << "baglaniyor" << std::endl;
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::TwistStamped twist;
    twist.twist.linear.x = 25;
    twist.twist.linear.y = -25;
    twist.twist.linear.z = 0;

    /*double roll = 30.0;
    double pitch = -20.0;

    double rolldeg = roll*3.14/180;
    double pitchdeg = pitch*3.14/180;

    tf2::Quaternion quaternion;
    quaternion.setRPY(rolldeg, pitchdeg, 0);

    double x = quaternion.x();
    double y = quaternion.y();
    double z = quaternion.z();
    double w = quaternion.w();

    mavros_msgs::AttitudeTarget attitude_setpoint;
    attitude_setpoint.orientation.x = x;
    attitude_setpoint.orientation.y = y;
    attitude_setpoint.orientation.z = z;
    attitude_setpoint.thrust = 1.0;
    attitude_setpoint.orientation.w = w; // Quaternion scalar (set to one for planes)*/
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_vel_pub.publish(twist);
        //local_pos_pub.publish(pose);
        //attitude_pub.publish(attitude_setpoint);
        //cmd_client.call(cmd_msg);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    mavros_msgs::ParamValue vel_const;
    vel_const.real = 17.31;
    
    mavros_msgs::ParamSet offb_param_set;
    offb_param_set.request.param_id = "FW_AIRSPD_TRIM";
    offb_param_set.request.value = vel_const;
    
    set_param_vel.call(offb_param_set);
    ros::Duration(3.0).sleep();

    ros::Time last_request = ros::Time::now();
    
    takeoffClient.waitForExistence();
    
    
    mavros_msgs::CommandTOL takeoffCommand;
    takeoffCommand.request.altitude = 20.0; // Set desired altitude in meters
    
    
    arming_client.call(arm_cmd);
    ros::Duration(3.0).sleep();
    
    
    takeoffClient.call(takeoffCommand);
    // Wait for the takeoff to complete
    ros::Duration(10.0).sleep();
    

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        //local_vel_pub.publish(twist);
        //local_pos_pub.publish(pose);
        //attitude_pub.publish(attitude_setpoint);
        //cmd_client.call(cmd_msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
