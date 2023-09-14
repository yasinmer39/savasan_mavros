#include <controller.hpp>

/*UÇUŞ KONTROLCÜLERİ ÇAĞIRMA FONKSİYONLARI*/
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

void MyClass::takeoff(double alt, ros::ServiceClient &arming_client, ros::ServiceClient &takeoffClient)
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

void MyClass::velocityInput(double x, double y, double z, ros::Publisher &local_vel_pub, ros::ServiceClient &set_param_vel)
{
    mavros_msgs::ParamValue vel_const;
    vel_const.real = sqrt(x*x + y*y);
    mavros_msgs::ParamSet offb_param_set;
    offb_param_set.request.param_id = "FW_AIRSPD_TRIM";
    offb_param_set.request.value = vel_const;

    geometry_msgs::TwistStamped twist;
    twist.twist.linear.x = x;
    twist.twist.linear.y = y;
    twist.twist.linear.z = z;

    local_vel_pub.publish(twist);
    set_param_vel.call(offb_param_set);
    
    ros::spinOnce();
  
}

void MyClass::orbit(float radius, float velocity, float yaw_behavior, float orbits, float lat, float lon, float alt, float xOffsetMeters, float yOffsetMeters, ros::ServiceClient &cmd_client)
{

    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;

    double latDistance = yOffsetMeters / R;
    double lonDistance = xOffsetMeters / (R * cos(lat_rad));

    double newLatRad = lat_rad + latDistance;
    double newLonRad = lon_rad + lonDistance;

    mavros_msgs::CommandLong cmd_msg;
    cmd_msg.request.command = 34;
    cmd_msg.request.param1 = radius;
    cmd_msg.request.param2 = velocity;
    cmd_msg.request.param3 = yaw_behavior;
    cmd_msg.request.param4 = orbits;
    cmd_msg.request.param5 = newLatRad * 180.0 / M_PI;
    cmd_msg.request.param6 = newLonRad * 180.0 / M_PI;
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
    ros::spinOnce();

}

/*UÇUŞ MODU AYARLAMA FONKSİYONLARI*/
void MyClass::setOffboard(mavros_msgs::State &current_state, ros::ServiceClient &set_mode_client)
{
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    if(current_state.mode != "OFFBOARD"){
        if(set_mode_client.call(offb_set_mode)){
            ROS_INFO("Ofbboard enabled");
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

/*CALLBACK FONKSİYONLARI*/
void MyClass::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &gps_msg)
{

    gps_lat = gps_msg->latitude;
    gps_lon = gps_msg->longitude;
    gps_alt = gps_msg->altitude;
}

void MyClass::state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void MyClass::odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{

    x = odom_msg->pose.pose.position.x;
    y = odom_msg->pose.pose.position.y;
    z = odom_msg->pose.pose.position.z;
    ros::spinOnce();
}

void MyClass::serverRead(ros::ServiceClient &stringServiceClient)
{

    try
    {
        if (stringServiceClient.call(srv))
        {
            ROS_INFO("Received string data: %s", srv.response.response_data.c_str());
            nlohmann::json jsonData = nlohmann::json::parse(srv.response.response_data.c_str());

            for (const auto &item : jsonData)
            {

                std::vector<float> team_info;

                team_info.push_back(item["takim_numarasi"]);
                team_info.push_back(item["iha_enlem"]);
                team_info.push_back(item["iha_boylam"]);
                team_info.push_back(item["iha_irtifa"]);
                team_info.push_back(item["iha_dikilme"]);
                team_info.push_back(item["iha_yonelme"]);
                team_info.push_back(item["iha_yatis"]);
                team_info.push_back(item["iha_x"]);
                team_info.push_back(item["iha_y"]);
                team_info.push_back(item["iha_z"]);
                team_info.push_back(item["zaman_farki"]);

                teams_info.push_back(team_info);
            }
        }
        else
        {
            ROS_ERROR("Failed to call rosservice");
        }
    }
    catch (ros::Exception &e)
    {
        ROS_ERROR("Service call failed: %s", e.what());
    }

    std::cout << "request" << std::endl;
}

void MyClass::metersToLatitudeLongitude(double originLat, double originLon, double xOffsetMeters, double yOffsetMeters, double &newLat, double &newLon)
{

    double originLatRad = originLat * M_PI / 180.0;
    double originLonRad = originLon * M_PI / 180.0;

    double latDistance = yOffsetMeters / R;
    double lonDistance = xOffsetMeters / (R * cos(originLatRad));

    double newLatRad = originLatRad + latDistance;
    double newLonRad = originLonRad + lonDistance;

    newLat = newLatRad * 180.0 / M_PI;
    newLon = newLonRad * 180.0 / M_PI;
}

// MAV_CMD_NAV_LOITER_TO_ALT belli bir yüksekliğe loiter atarak yükseliyor kaçış için kullanılabilir.