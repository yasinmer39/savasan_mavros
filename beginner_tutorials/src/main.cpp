#include "controller.hpp"

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg) {

    lat = gps_msg->latitude;
    lon = gps_msg->longitude;
    alt = gps_msg->altitude;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    ("uav0/mavros/state", 10, state_cb);
    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>
    ("uav0/mavros/global_position/global", 10, gpsCallback);
    ros::Publisher attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>
    ("uav0/mavros/setpoint_raw/attitude", 10);
    ros::ServiceClient cmd_client = nh.serviceClient<mavros_msgs::CommandLong>
    ("uav0/mavros/cmd/command", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
    ("uav0/mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient takeoffClient = nh.serviceClient<mavros_msgs::CommandTOL>
    ("uav0/mavros/cmd/takeoff");
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    ("uav0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    ("uav0/mavros/set_mode");
    ros::ServiceClient set_param_vel = nh.serviceClient<mavros_msgs::ParamSet>
    ("uav0/mavros/param/set");
    ros::Duration(2.0).sleep();
    paramSet(15.0, set_mode_client, set_param_vel);
    for(int i=0; i<10000000; ++i){ros::spinOnce();}
    takeoff(30.0, arming_client, takeoffClient);
    orbit(200.0, 15.0, 0.0, 0.0, lat, lon, alt, cmd_client);

    while(ros::ok){
        //setOffboard(current_state, set_mode_client);
        //attitude(10.0, 0.0, 0.5, attitude_pub);
        //cmd_client.call(mavros_data.cmd_msg);
        //orbit(50.0, 20.0, 0.0, 0.0, lat, lon, 40.0, cmd_client);
        
        ros::spinOnce();
    }

    return 0;
}