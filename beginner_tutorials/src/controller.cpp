#include <controller.hpp>

void takeoff(double alt, ros::ServiceClient& arming_client, ros::ServiceClient& takeoffClient){

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    mavros_msgs::CommandTOL takeoffCommand;
    takeoffCommand.request.altitude = alt;

    takeoffClient.waitForExistence();
    arming_client.call(arm_cmd);
    takeoffClient.call(takeoffCommand);
    ros::Duration(10.0).sleep();
}

void velocityInput(double x, double y, double z, ros::Publisher& local_vel_pub){

    geometry_msgs::TwistStamped twist;
    twist.twist.linear.x = x;
    twist.twist.linear.y = y;
    twist.twist.linear.z = z;

    local_vel_pub.publish(twist);
 
}

void orbit(float radius, float velocity, float yaw_behavior, float orbits, float lat, float lon, float alt, ros::ServiceClient& cmd_client){
    
    /*constexpr double R = 6371000.0;
    double lat1 = 180.0*(asin(sin(M_PI*(lat)/180.0) * cos(distance / R) + cos(M_PI*(lat)/180.0) * sin(distance / R) * cos(0)))/M_PI;
    double lon1 = 180.0*(M_PI*(lon)/180.0 + atan2(sin(0) * sin(distance / R) * cos(M_PI*(lat)/180.0), cos(distance / R) - sin(M_PI*(lat)/180.0) * sin(M_PI*(lat1)/180.0)))/M_PI;
    */
    //MAVROS mavros_data;
    mavros_msgs::CommandLong cmd_msg;
    cmd_msg.request.command = 34; // MAV_CMD_NAV_LOITER_TURNS 75
    cmd_msg.request.param1 = radius;
    cmd_msg.request.param2 = velocity; 
    cmd_msg.request.param3 = yaw_behavior;
    cmd_msg.request.param4 = orbits; 
    cmd_msg.request.param5 = lat; // lat
    cmd_msg.request.param6 = lon; // lon
    cmd_msg.request.param7 = alt; // alt

    cmd_client.call(cmd_msg);
    //ros::Duration(4.0).sleep();

}

void attitude(double roll, double pitch, float thrust, ros::Publisher& attitude_pub){

    tf2::Quaternion quaternion;
    quaternion.setRPY(roll*M_PI/180, -1*pitch*M_PI/180, 0);

    mavros_msgs::AttitudeTarget attitude_setpoint;
    attitude_setpoint.orientation.x = quaternion.x();
    attitude_setpoint.orientation.y = quaternion.y();
    attitude_setpoint.orientation.z = quaternion.z();
    attitude_setpoint.orientation.w = quaternion.w();
    attitude_setpoint.thrust = thrust; /*thrust 0-1 arasında*/

    attitude_pub.publish(attitude_setpoint);

}

//uçak offboardda mı diye kontrol eder, değilse offboarda alır.
void setOffboard(mavros_msgs::State& current_state, ros::ServiceClient& set_mode_client){

    if(current_state.mode != "OFFBOARD"){mavros_msgs::SetMode set_mode;
        set_mode.request.custom_mode = "OFFBOARD";
        if (set_mode_client.call(set_mode) && set_mode.response.mode_sent) {
                ROS_INFO("Switched to OFFBOARD mode");
            } else {
                ROS_ERROR("Failed to switch to OFFBOARD mode");
            }
    }
    
}

//hold modda mı kontrol et, değilse hold moda at.
void holdMode(ros::Subscriber& state_sub, ros::ServiceClient& set_mode_client){

    mavros_msgs::State current_state;
    mavros_msgs::SetMode hold_set_mode;
    if(current_state.mode == "HOLD"){
        ROS_INFO("HOLD MODE");
    }
    else{
        hold_set_mode.request.custom_mode = "HOLD";
        set_mode_client.call(hold_set_mode);
    }

}

void paramSet(double vel, ros::ServiceClient& set_mode_client, ros::ServiceClient& set_param_vel){

    mavros_msgs::ParamValue vel_const;
    vel_const.real = vel; 
    mavros_msgs::ParamSet offb_param_set;
    offb_param_set.request.param_id = "FW_AIRSPD_TRIM";
    offb_param_set.request.value = vel_const;
    /*offb_param_set.request.param_id = "REL_ALT";
    offb_param_set.request.value.real = alt*/
    set_param_vel.call(offb_param_set);

}


//MAV_CMD_NAV_LOITER_TO_ALT belli bir yüksekliğe loiter atarak yükseliyor kaçış için kullanılabilir.