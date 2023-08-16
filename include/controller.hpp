#pragma once

#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamValue.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <tf2/LinearMath/Quaternion.h> 
#include <mavros_msgs/GlobalPositionTarget.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/NavSatFix.h>

class MyClass{

    public:
        MyClass() = delete;
        MyClass(ros::NodeHandle &nodehandle);
        MyClass(MyClass& myclass) =default;
        MyClass(MyClass&& rhs) = default;
        ~MyClass() = default;

        void takeoff(double alt, ros::ServiceClient& arming_client, ros::ServiceClient& takeoffClient);
        void velocityInput(double x, double y, double z, ros::Publisher& local_vel_pub);
        void orbit(float radius, float velocity, float yaw_behavior, float orbits, float lat, float lon, float alt, ros::ServiceClient& cmd_client);
        void attitude(double roll, double pitch, float thrust, ros::Publisher& attitude_pub);
        void setOffboard(mavros_msgs::State& current_state, ros::ServiceClient& set_mode_client);
        void holdMode(mavros_msgs::State &current_state, ros::ServiceClient &hold_set_mode_client);
        void paramSet(double vel, ros::ServiceClient& set_mode_client, ros::ServiceClient& set_param_vel);
        void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg);
        void state_cb(const mavros_msgs::State::ConstPtr& msg);

        ros::Subscriber state_sub;
        ros::Subscriber gps_sub;
        ros::Publisher attitude_pub;
        ros::ServiceClient cmd_client;
        ros::Publisher local_vel_pub;
        ros::ServiceClient takeoffClient;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        ros::ServiceClient set_param_vel;
        ros::NodeHandle  nh;
        mavros_msgs::State current_state;

        double lat, lon, alt;

};



