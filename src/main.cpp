#include "controller.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    
    MyClass myclass{nh};

    myclass.paramSet(16.0, myclass.set_mode_client, myclass.set_param_vel);
    myclass.takeoff(30.0, myclass.arming_client, myclass.takeoffClient);
    myclass.orbit(200.0, 15.0, 0.0, 0.0, myclass.lat, myclass.lon, myclass.alt, myclass.cmd_client);

    while(ros::ok){
    
        ros::spinOnce();
    }

    return 0;
}