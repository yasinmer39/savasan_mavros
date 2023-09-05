#include "controller.hpp"

bool shouldExit = false;

void signalHandler(int sig) {
    shouldExit = true;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    MyClass myclass{nh};
    signal(SIGINT, signalHandler);

    std::thread uav0_param(&MyClass::paramSet, &myclass, 16.0, std::ref(myclass.set_mode_client), std::ref(myclass.set_param_vel));
    std::thread uav1_param(&MyClass::paramSet, &myclass, 16.0, std::ref(myclass.set_mode_client1), std::ref(myclass.set_param_vel1));
    std::thread uav0_to(&MyClass::takeoff, &myclass, 30.0, std::ref(myclass.arming_client), std::ref(myclass.takeoffClient));
    std::thread uav1_to(&MyClass::takeoff, &myclass, 30.0, std::ref(myclass.arming_client1), std::ref(myclass.takeoffClient1));
    
    uav0_param.join();
    uav1_param.join();
    uav0_to.join();
    uav1_to.join();

    myclass.metersToLatitudeLongitude(myclass.lat, myclass.lon, 200.0, 200.0, myclass.newLatitude, myclass.newLongitude);
    myclass.orbit(200.0, 15.0, 0.0, 0.0, myclass.newLatitude, myclass.newLongitude, myclass.alt, myclass.cmd_client);
    
    while(ros::ok && shouldExit){

        ros::spinOnce();
        
    }

    return 0;

}