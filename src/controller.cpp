#include <controller.hpp>

MyClass::MyClass(ros::NodeHandle &nodehandle) : nh(nodehandle)
{

    state_sub = nh.subscribe<mavros_msgs::State>("uav0/mavros/state", 10, &MyClass::state_cb, this);
    gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("uav0/mavros/global_position/global", 10, &MyClass::gpsCallback, this);
    attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("uav0/mavros/setpoint_raw/attitude", 10);
    cmd_client = nh.serviceClient<mavros_msgs::CommandLong>("uav0/mavros/cmd/command", 10);
    local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("uav0/mavros/setpoint_velocity/cmd_vel", 10);
    takeoffClient = nh.serviceClient<mavros_msgs::CommandTOL>("uav0/mavros/cmd/takeoff");
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("uav0/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("uav0/mavros/set_mode");
    set_param_vel = nh.serviceClient<mavros_msgs::ParamSet>("uav0/mavros/param/set");

}

void MyClass::takeoff(double alt , ros::ServiceClient &arming_client, ros::ServiceClient &takeoffClient)
{

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    mavros_msgs::CommandTOL takeoffCommand;
    takeoffCommand.request.altitude = alt;

    while (arm_cmd.response.success != true && takeoffCommand.response.success != true)
    {
        arming_client.call(arm_cmd);
        takeoffClient.call(takeoffCommand);
        ros::Duration(1.5).sleep();
        ros::spinOnce();
        if (arm_cmd.response.success == true && takeoffCommand.response.success == true)
        {
            ROS_INFO("Armed and Takeoff Command sent.");
        }
    }

    ros::Duration(6).sleep();

}

void MyClass::velocityInput(double x, double y, double z, ros::Publisher &local_vel_pub)
{

    geometry_msgs::TwistStamped twist;
    twist.twist.linear.x = x;
    twist.twist.linear.y = y;
    twist.twist.linear.z = z;

    local_vel_pub.publish(twist);

}

void MyClass::orbit(float radius, float velocity, float yaw_behavior, float orbits, float lat, float lon, float alt, ros::ServiceClient &cmd_client)
{

    /*constexpr double R = 6371000.0;
    double lat1 = 180.0*(asin(sin(M_PI*(lat)/180.0) * cos(distance / R) + cos(M_PI*(lat)/180.0) * sin(distance / R) * cos(0)))/M_PI;
    double lon1 = 180.0*(M_PI*(lon)/180.0 + atan2(sin(0) * sin(distance / R) * cos(M_PI*(lat)/180.0), cos(distance / R) - sin(M_PI*(lat)/180.0) * sin(M_PI*(lat1)/180.0)))/M_PI;
    */
    mavros_msgs::CommandLong cmd_msg;
    cmd_msg.request.command = 34;
    cmd_msg.request.param1 = radius;
    cmd_msg.request.param2 = velocity;
    cmd_msg.request.param3 = yaw_behavior;
    cmd_msg.request.param4 = orbits;
    cmd_msg.request.param5 = lat;
    cmd_msg.request.param6 = lon;
    cmd_msg.request.param7 = alt;

    cmd_client.call(cmd_msg);

}

void MyClass::attitude(double roll, double pitch, float thrust, ros::Publisher &attitude_pub)
{

    tf2::Quaternion quaternion;
    quaternion.setRPY(roll * M_PI / 180, -1 * pitch * M_PI / 180, 0);

    mavros_msgs::AttitudeTarget attitude_setpoint;
    attitude_setpoint.orientation.x = quaternion.x();
    attitude_setpoint.orientation.y = quaternion.y();
    attitude_setpoint.orientation.z = quaternion.z();
    attitude_setpoint.orientation.w = quaternion.w();
    attitude_setpoint.thrust = thrust; /*thrust 0-1 arasında*/

    attitude_pub.publish(attitude_setpoint);
}

void MyClass::setOffboard(mavros_msgs::State &current_state, ros::ServiceClient &set_mode_client)
{

    if (current_state.mode != "OFFBOARD")
    {
        mavros_msgs::SetMode set_mode;
        set_mode.request.custom_mode = "OFFBOARD";
        if (set_mode_client.call(set_mode) && set_mode.response.mode_sent)
        {
            ROS_INFO("Switched to OFFBOARD mode");
        }
        else
        {
            ROS_ERROR("Failed to switch to OFFBOARD mode");
        }
    }
}

void MyClass::holdMode(mavros_msgs::State &current_state, ros::ServiceClient &hold_set_mode_client)
{

    if (current_state.mode != "HOLD")
    {
        mavros_msgs::SetMode hold_set_mode;
        hold_set_mode.request.custom_mode = "HOLD";
        if (hold_set_mode_client.call(hold_set_mode) && hold_set_mode.response.mode_sent)
        {
            ROS_INFO("Switched to OFFBOARD mode");
        }
        else
        {
            ROS_ERROR("Failed to switch to OFFBOARD mode");
        }
    }
}

void MyClass::paramSet(double vel, ros::ServiceClient &set_mode_client, ros::ServiceClient &set_param_vel)
{

    mavros_msgs::ParamValue vel_const;
    vel_const.real = vel;
    mavros_msgs::ParamSet offb_param_set;
    offb_param_set.request.param_id = "FW_AIRSPD_TRIM";
    offb_param_set.request.value = vel_const;

    while (offb_param_set.response.success != true)
    {
        set_param_vel.call(offb_param_set);
        ros::Duration(1.5).sleep();
        ros::spinOnce();
        if (offb_param_set.response.success == true)
        {
            ROS_INFO("Velocity parameter set successfully.");
        }
    }
}

void MyClass::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &gps_msg)
{

    lat = gps_msg->latitude;
    lon = gps_msg->longitude;
    alt = gps_msg->altitude;

}

void MyClass::state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

// MAV_CMD_NAV_LOITER_TO_ALT belli bir yüksekliğe loiter atarak yükseliyor kaçış için kullanılabilir.