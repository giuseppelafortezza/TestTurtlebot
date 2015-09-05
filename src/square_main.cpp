#include "ros/ros.h"
#include <sstream>
#include <math.h>
#include "controller/square_controller.h"


    
using namespace std;

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "square_controller");
    square_controller cont;
    cont.init();
    cont.run();

    return 0;
}