#include "controller.hpp"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "offb_node");
    
    ros::NodeHandle nh;

        try
    {
        // Create the rosservice client
        ros::ServiceClient stringServiceClient = nh.serviceClient<beginner_tutorials::Telemetry>("/string_service");
        beginner_tutorials::Telemetry srv;

        // Send the request to the rosservice server and receive the response
        if (stringServiceClient.call(srv))
        {
            ROS_INFO("Received string data: %s", srv.response.response_data.c_str());
        }
        else
        {
            ROS_ERROR("Failed to call rosservice");
        }
    }
    catch (ros::Exception& e)
    {
        ROS_ERROR("Service call failed: %s", e.what());
    }


    MyClass uav0{nh, 0};
    MyClass uav1{nh, 1};
    MyClass uav2{nh, 2};
    MyClass uav3{nh, 3};
    MyClass uav4{nh, 4};
    MyClass uav5{nh, 5};
    MyClass uav6{nh, 6};
    MyClass uav7{nh, 7};
    MyClass uav8{nh, 8};
    MyClass uav9{nh, 9};


    std::thread uav0_param(&MyClass::paramSet, &uav0, 16.0, std::ref(uav0.set_mode_client), std::ref(uav0.set_param_vel));
    std::thread uav1_param(&MyClass::paramSet, &uav1, 16.0, std::ref(uav1.set_mode_client), std::ref(uav1.set_param_vel));
    uav0_param.join();
    uav1_param.join();
    std::thread uav0_to(&MyClass::takeoff, &uav0, 30.0, std::ref(uav0.arming_client), std::ref(uav0.takeoffClient));
    std::thread uav1_to(&MyClass::takeoff, &uav1, 30.0, std::ref(uav1.arming_client), std::ref(uav1.takeoffClient));
    uav0_to.join();
    uav1_to.join();
    std::thread uav0_orbit(&MyClass::orbit, &uav0, 200.0, 15.0, 0.0, 0.0, uav0.gps_lat, uav0.gps_lon, uav0.gps_alt, 200.0, 200.0, std::ref(uav0.cmd_client));
    std::thread uav1_orbit(&MyClass::orbit, &uav1, 200.0, 15.0, 0.0, 0.0, uav1.gps_lat, uav1.gps_lon, uav1.gps_alt, -200.0, -200.0, std::ref(uav1.cmd_client));
    uav0_orbit.join();
    uav1_orbit.join();
    ros::Rate ros_sleep{10};
    while (ros::ok)
    {

        ros::spinOnce();
        ros_sleep.sleep();
    }

    return 0;
}