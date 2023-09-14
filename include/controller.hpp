#pragma once

#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <cmath>
#include <thread>
#include <mutex>
#include <vector>
#include <cstdlib>
#include <condition_variable>
#include <nlohmann/json.hpp>
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
#include <nav_msgs/Odometry.h>
#include <beginner_tutorials/Telemetry.h>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

class MyClass{

    private:

        int id;
        std::string base_topic;

    public:

        MyClass() = delete;
        MyClass(ros::NodeHandle &nodehandle, int id) : nh(nodehandle), id(id){

            base_topic = "uav" + std::to_string(id);

            state_sub       = nh.subscribe<mavros_msgs::State>(getState(), 10, &MyClass::state_cb, this);
            gps_sub         = nh.subscribe<sensor_msgs::NavSatFix>(getGPS(), 10, &MyClass::gpsCallback, this);
            odom_sub        = nh.subscribe<nav_msgs::Odometry>(getOdom(), 10, &MyClass::odomCallback, this);

            attitude_pub    = nh.advertise<mavros_msgs::AttitudeTarget>(getAtt(), 10);
            local_vel_pub   = nh.advertise<geometry_msgs::TwistStamped>(getVel(), 10);

            cmd_client      = nh.serviceClient<mavros_msgs::CommandLong>(getCL(), 10);
            takeoffClient   = nh.serviceClient<mavros_msgs::CommandTOL>(getCT());
            arming_client   = nh.serviceClient<mavros_msgs::CommandBool>(getCB());
            set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(getMode());
            set_param_vel   = nh.serviceClient<mavros_msgs::ParamSet>(getParam());
            stringServiceClient = nh.serviceClient<beginner_tutorials::Telemetry>("/string_service");
            
        };
        MyClass(MyClass& myclass) = default;
        MyClass(MyClass&& rhs) = default;
        ~MyClass() = default;

        std::string getState(){
            return base_topic + "/mavros/state";
        }
        std::string getGPS(){
            return base_topic + "/mavros/global_position/global";
        }
        std::string getOdom(){
            return base_topic + "/mavros/local_position/odom";
        }
        std::string getAtt(){
            return base_topic + "/mavros/setpoint_raw/attitude";
        }
        std::string getVel(){
            return base_topic + "/mavros/setpoint_velocity/cmd_vel";
        }
        std::string getCL(){
            return base_topic + "/mavros/cmd/command";
        }
        std::string getCT(){
            return base_topic + "/mavros/cmd/takeoff";
        }
        std::string getCB(){
            return base_topic + "/mavros/cmd/arming";
        }
        std::string getMode(){
            return base_topic + "/mavros/set_mode";
        }
        std::string getParam(){
            return base_topic + "/mavros/param/set";
        }
      
        void takeoff(double alt, ros::ServiceClient& arming_client, ros::ServiceClient& takeoffClient);
        void velocityInput(double x, double y, double z, ros::Publisher& local_vel_pub, ros::ServiceClient &set_param_vel);
        void orbit(float radius, float velocity, float yaw_behavior, float orbits, float lat, float lon, float alt, float xOffsetMeters, float yOffsetMeters, ros::ServiceClient& cmd_client);
        void attitude(double roll, double pitch, float thrust, ros::Publisher& attitude_pub);
        void setOffboard(mavros_msgs::State& current_state, ros::ServiceClient& set_mode_client);
        void holdMode(mavros_msgs::State &current_state, ros::ServiceClient &hold_set_mode_client);
        void paramSet(double vel, ros::ServiceClient& set_mode_client, ros::ServiceClient& set_param_vel);
        void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg);
        void state_cb(const mavros_msgs::State::ConstPtr& msg);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
        void metersToLatitudeLongitude(double originLat, double originLon, double xOffsetMeters, double yOffsetMeters, double& newLat, double& newLon);
        void serverRead(ros::ServiceClient &stringServiceClient);
        void dogfightVector(double uav0_lat, double uav0_lon, double uav1_lat, double uav1_lon);

        ros::Subscriber state_sub;
        ros::Subscriber gps_sub;
        ros::Subscriber odom_sub;
        ros::Publisher attitude_pub;
        ros::Publisher local_vel_pub;
        ros::ServiceClient cmd_client;
        ros::ServiceClient takeoffClient;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        ros::ServiceClient set_param_vel;
        ros::ServiceClient stringServiceClient;
        ros::NodeHandle  nh;
        mavros_msgs::State current_state;
        beginner_tutorials::Telemetry srv;

        std::vector<std::vector<float>> teams_info;

        double lat, lon, alt, x, y, z, newLatitude, newLongitude, gps_lat, gps_lon, gps_alt;
        const double R = 6371000.0;
 
};



