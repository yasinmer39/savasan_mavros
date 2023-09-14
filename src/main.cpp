#include "controller.hpp"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "offb_node");

    ros::NodeHandle nh;

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

    uav0.serverRead(uav0.stringServiceClient);
    uav1.serverRead(uav1.stringServiceClient);

    for (uint8_t i=0; i < uav0.teams_info.size(); i++)
    {
    
        for (uint8_t j=0; j < uav0.teams_info[i].size(); j++)
        {
            std::cout << uav0.teams_info[i][j] << std::endl;
        }
        std::cout << std::endl;

    }

    float server0_lat = uav0.teams_info[0][1];
    float server1_lon = uav1.teams_info[1][1];

    std::thread uav0_param(&MyClass::paramSet, &uav0, 20.0, std::ref(uav0.set_mode_client), std::ref(uav0.set_param_vel));
    std::thread uav1_param(&MyClass::paramSet, &uav1, 20.0, std::ref(uav1.set_mode_client), std::ref(uav1.set_param_vel));
    uav0_param.join();
    uav1_param.join();
    std::thread uav0_to(&MyClass::takeoff, &uav0, 30.0, std::ref(uav0.arming_client), std::ref(uav0.takeoffClient));
    std::thread uav1_to(&MyClass::takeoff, &uav1, 30.0, std::ref(uav1.arming_client), std::ref(uav1.takeoffClient));
    uav0_to.join();
    uav1_to.join();   
    std::thread uav0_orbit(&MyClass::orbit, &uav0, 200.0, 15.0, 0.0, 1.0, 31.0, uav0.gps_lon, uav0.gps_alt, 200.0, 200.0, std::ref(uav0.cmd_client));
    std::thread uav1_orbit(&MyClass::orbit, &uav1, 200.0, 15.0, 0.0, 1.0, 31.0, uav1.gps_lon, uav1.gps_alt, -200.0, -200.0, std::ref(uav1.cmd_client));
    uav0_orbit.join();
    uav1_orbit.join();
    uav0.setOffboard(uav0.current_state, uav0.set_mode_client);
    uav1.setOffboard(uav1.current_state, uav1.set_mode_client);

    while (ros::ok)
    {    
 
        uav0.setOffboard(uav0.current_state, uav0.set_mode_client);
        uav1.setOffboard(uav1.current_state, uav1.set_mode_client);

        uav1.velocityInput(10.0, 10.0, 0.0, uav1.local_vel_pub, uav1.set_param_vel);

        uav0.attitude(30.0, 0.0, 1.0, uav0.attitude_pub);
        

        ros::spinOnce();

    }

    return 0;
}