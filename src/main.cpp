#include "ros/ros.h"
#include "controller/controller.h"
#include <sstream>
#include <math.h>


    
using namespace std;

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "controller");
    controller cont;
    cont.init();
    cont.run();

    return 0;
}