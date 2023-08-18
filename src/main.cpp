#include "controller.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    
    MyClass myclass{nh};

    myclass.paramSet(16.0, myclass.set_mode_client, myclass.set_param_vel);
    myclass.takeoff(30.0, myclass.arming_client, myclass.takeoffClient);
    myclass.metersToLatitudeLongitude(myclass.lat, myclass.lon, 200.0, 200.0, myclass.newLatitude, myclass.newLongitude);
    myclass.orbit(200.0, 15.0, 0.0, 0.0, myclass.newLatitude, myclass.newLongitude, myclass.alt, myclass.cmd_client);
    std::cout<<"old lat "<<myclass.lat<<", old lon " <<myclass.lon<<std::endl;
    std::cout<<"NEW lat "<<myclass.newLatitude<<", NEW lon " <<myclass.newLongitude<<std::endl;
    while(ros::ok){
    
        ros::spinOnce();
        
    }

    return 0;
}