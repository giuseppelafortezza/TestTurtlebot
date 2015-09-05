#include "ros/ros.h"
#include <sstream>
#include <math.h>
#include "../include/controller/reference_controller.h"


    
using namespace std;

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "reference_controller");
    reference_controller cont;
    cont.init();
    cont.run();

    return 0;
}